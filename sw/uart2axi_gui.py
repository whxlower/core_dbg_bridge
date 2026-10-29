#!/usr/bin/env python
# -*- coding: utf-8 -*-
#
# Copyright (C) 2026 whx <253616211@qq.com>
#
# This file is part of an unofficial, modified fork of the UART -> AXI Debug
# Bridge (https://github.com/ultraembedded/dbg_bridge), Copyright (C) 2017-2019
# Ultra-Embedded.com, and is distributed under the GNU Lesser General Public
# License, version 2.1 or (at your option) any later version.
#
# See the LICENSE and NOTICE files in the repository root. This program is
# distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY;
# without even the implied warranty of MERCHANTABILITY or FITNESS FOR A
# PARTICULAR PURPOSE.
#
##################################################################
# uart2axi_gui.py: UART -> AXI 32bit 寄存器调试工具（图形界面）
#
# 线上协议与 uart_bus_interface.py / socket_interface.py 完全一致：
#   写命令: 0x10, len, addr[31:24], addr[23:16], addr[15:8], addr[7:0], data[0..len-1]
#   读命令: 0x11, len, addr[31:24], addr[23:16], addr[15:8], addr[7:0]
#   读回复: len 字节
#   地址: 固定 32bit 大端(MSB first)；数据: 小端(LSB first)
# 所有访问统一使用变长接口 read()/write()，不使用 read32()/write32()。
# GUI 布局：串口设置 / 寄存器访问（含可编辑的 GPIO、检测预设地址与快捷填入）/ 通信日志
##################################################################
import argparse
import ast
import math
import os
import queue
import sys
import threading
import time
import tkinter as tk
from tkinter import filedialog, messagebox
from tkinter import ttk

import serial
from serial.tools import list_ports

from uart_bus_interface import UartBusInterface

##################################################################
# 协议常量（与 UartBusInterface 中的定义保持一致）
##################################################################
CMD_WRITE  = 0x10
CMD_READ   = 0x11
GPIO_ADDR  = 0xF0000000
STS_ADDR   = 0xF0000004
STS_MAGIC  = 0xCAFE0000

BAUD_RATES = ['9600', '19200', '38400', '57600', '115200', '230400',
              '460800', '921600', '1000000', '1500000', '2000000', '3000000']
WIDTHS     = (1, 2, 4)
WIDTH_TEXT = {1: '1 字节', 2: '2 字节', 4: '4 字节'}

BATCH_BLOCK_WORDS = 32          # 连续读写时每帧的字数（默认 32 字 = 128 字节）
BATCH_BLOCK_MAX   = 63          # 每帧最多 63 字 = 252 字节（协议单帧上限 255 字节）


##################################################################
# 脚本解析: 每行一次 4 字节操作，格式 "<addr>:<data>"（均为 16 进制）
##################################################################
class ScriptError(ValueError):
    """脚本格式/执行错误；line_no 为 1 起行号（0 表示与具体行无关）"""

    def __init__(self, line_no, message):
        self.line_no = line_no
        self.message = message
        ValueError.__init__(self, ('第 %d 行: %s' % (line_no, message)) if line_no else message)


SCRIPT_WRITE = 'W'
SCRIPT_READ  = 'R'


def _script_value(text, line_no, name):
    """把 16 进制字段解析成 32bit 整数（允许 0x 前缀，最多 8 位，高位自动补 0）"""
    raw = text.strip()
    if raw[:2].lower() == '0x':
        raw = raw[2:]
    if not raw:
        raise ScriptError(line_no, '%s为空' % name)
    if len(raw) > 8:
        raise ScriptError(line_no, '%s超过 8 位 16 进制（最大 0xFFFFFFFF）' % name)
    try:
        value = int(raw, 16)
    except ValueError:
        raise ScriptError(line_no, '%s "%s" 不是合法的 16 进制数' % (name, text.strip()))
    if value < 0 or value > 0xFFFFFFFF:
        raise ScriptError(line_no, '%s超出 32bit 范围（0 ~ 0xFFFFFFFF）' % name)
    return value


def parse_script(text):
    """
    解析脚本内容，返回 (ops, corrected)。

    ops      : [{'line','op','raw_addr','addr','value','aligned'}, ...]
               op 为 SCRIPT_WRITE(有 data) 或 SCRIPT_READ(无 data)
    corrected: 地址未 4 字节对齐、被向下对齐的操作条数

    规则: 每行 "<addr>:<data>"，均为 16 进制；有 data 为写入，无 data（"addr:"）为读取；
          一律 4 字节操作；空行与 # 注释（含行尾注释）忽略。
    """
    ops       = []
    corrected = 0
    for line_no, raw_line in enumerate(text.splitlines(), 1):
        line = raw_line.split('#', 1)[0].strip()
        if not line:
            continue

        colon = -1
        for sep in (':', '：'):
            pos = line.find(sep)
            if pos >= 0 and (colon < 0 or pos < colon):
                colon = pos
        if colon < 0:
            raise ScriptError(line_no, '缺少分隔符 ":"，应为 "<addr>:<data>" 或 "<addr>:"')

        addr_text = line[:colon].strip()
        data_text = line[colon + 1:].strip()
        if not addr_text:
            raise ScriptError(line_no, '地址为空')

        raw_addr = _script_value(addr_text, line_no, '地址')
        addr     = (raw_addr & ~3) & 0xFFFFFFFF
        aligned  = addr != raw_addr

        if data_text:
            value = _script_value(data_text, line_no, '数据')
            op    = SCRIPT_WRITE
        else:
            value = None
            op    = SCRIPT_READ

        if aligned:
            corrected += 1
        ops.append({'line': line_no, 'op': op, 'raw_addr': raw_addr,
                    'addr': addr, 'value': value, 'aligned': aligned})

    if not ops:
        raise ScriptError(0, '脚本为空（没有可执行的操作）')
    return ops, corrected


##################################################################
# 波表 / 批量数据文件
#   文件格式: 每行一个 32bit 数据，8 位大写 16 进制且不带 0x 前缀
#   生成: f(n) + O -> 四舍五入 -> 截断到 32bit 有符号 -> 8 位 16 进制
#   解析: 纯 0-9A-F（可带 0x）按 16 进制；带负号按 10 进制（-1 即 FFFFFFFF）
##################################################################
class WaveError(ValueError):
    """波表表达式 / 数据文件错误；line_no 为 1 起行号（0 表示与具体行无关）"""

    def __init__(self, message, line_no=0):
        self.line_no = line_no
        self.message = message
        ValueError.__init__(self, ('第 %d 行: %s' % (line_no, message)) if line_no else message)


class WaveCancelled(Exception):
    """波表生成 / 文件写入被用户取消"""


WAVE_MIN         = -0x80000000
WAVE_MAX         = 0x7FFFFFFF
WAVE_MASK        = 0xFFFFFFFF
WAVE_MAX_COUNT   = 1000000      # 单次生成/读写的最大点数
WAVE_STAT_SAMPLE = 65536        # 预览统计最多采样点数（避免大表卡界面）
WAVE_NAMES       = ('n', 'N', 'pi', 'e', 'A')

WAVE_TEMPLATES = (
    ('正弦',       'A*sin(2*pi*n/N)'),
    ('余弦',       'A*cos(2*pi*n/N)'),
    ('方波',       'A if n < N/2 else -A'),
    ('三角波',     'A*(4*abs(n/N-0.5)-1)'),
    ('锯齿波',     'A*(2*n/N-1)'),
    ('双正弦叠加', 'A*(sin(2*pi*n/N)+0.5*sin(4*pi*n/N))/1.5'),
    ('常数',       'A'),
)

WAVE_FUNCS = {
    'sin': math.sin, 'cos': math.cos, 'tan': math.tan,
    'asin': math.asin, 'acos': math.acos, 'atan': math.atan,
    'atan2': math.atan2, 'sqrt': math.sqrt, 'fabs': math.fabs,
    'exp': math.exp, 'log': math.log, 'log10': math.log10,
    'floor': math.floor, 'ceil': math.ceil, 'round': round,
    'sign': lambda value: (value > 0) - (value < 0),
    'abs': abs, 'min': min, 'max': max,
}

_WAVE_AST_OK = (
    ast.Expression, ast.BinOp, ast.UnaryOp, ast.Constant, ast.Name, ast.Load,
    ast.Call, ast.IfExp, ast.Compare, ast.BoolOp, ast.And, ast.Or,
    ast.Add, ast.Sub, ast.Mult, ast.Div, ast.FloorDiv, ast.Mod, ast.Pow,
    ast.UAdd, ast.USub, ast.Not, ast.Lt, ast.LtE, ast.Gt, ast.GtE, ast.Eq, ast.NotEq,
)


##################################################################
# check_wave_expr / compile_wave_expr: ast 白名单校验（禁止属性、下标、
# lambda、推导式、import 等一切可能越界的写法）
##################################################################
def check_wave_expr(expr):
    text = (expr or '').strip()
    if not text:
        raise WaveError('函数 f(n) 不能为空')
    try:
        tree = ast.parse(text, '<波表>', 'eval')
    except SyntaxError as exc:
        raise WaveError('表达式语法错误: %s' % (exc.msg or '无法解析'))

    for node in ast.walk(tree):
        if type(node) not in _WAVE_AST_OK:
            raise WaveError('不支持的写法: %s（只允许数字、n/N/pi/e/A、四则运算、'
                            '比较、and/or/not、a if c else b 和常用数学函数）'
                            % type(node).__name__)
        if isinstance(node, ast.Call):
            if not isinstance(node.func, ast.Name) or node.func.id not in WAVE_FUNCS:
                raise WaveError('不允许调用该函数（可用: %s）' % ' '.join(sorted(WAVE_FUNCS)))
        elif isinstance(node, ast.Name):
            if node.id not in WAVE_NAMES and node.id not in WAVE_FUNCS:
                raise WaveError('未知名称 "%s"（可用: %s）' % (node.id, ' '.join(WAVE_NAMES)))
        elif isinstance(node, ast.Constant):
            if isinstance(node.value, bool) or not isinstance(node.value, (int, float)):
                raise WaveError('常量 "%s" 不是数值' % (node.value,))
    return tree


def compile_wave_expr(expr):
    return compile(check_wave_expr(expr), '<波表>', 'eval')


##################################################################
# eval_wave_point: 计算单点，返回 (有符号 32bit 值, 是否被截断)
##################################################################
def _wave_round(value):
    """四舍五入（负数同样远离 0），不用 Python round() 的银行家舍入"""
    if value >= 0:
        return int(math.floor(value + 0.5))
    return -int(math.floor(-value + 0.5))


def eval_wave_point(code, n, count, amp, offset):
    env = {'__builtins__': {}, 'n': n, 'N': count, 'pi': math.pi,
           'e': math.e, 'A': amp}
    env.update(WAVE_FUNCS)
    try:
        value = eval(code, env)
    except ZeroDivisionError:
        raise WaveError('第 %d 点（n=%d）计算除零' % (n + 1, n))
    except (ValueError, OverflowError) as exc:
        raise WaveError('第 %d 点（n=%d）计算出错: %s' % (n + 1, n, exc))
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise WaveError('第 %d 点（n=%d）结果不是数值' % (n + 1, n))
    if isinstance(value, float):
        if math.isnan(value) or math.isinf(value):
            raise WaveError('第 %d 点（n=%d）不是有限数值（检查 log/sqrt/除法）'
                            % (n + 1, n))
        value = _wave_round(value)
    value += int(offset)
    if value < WAVE_MIN:
        return WAVE_MIN, True
    if value > WAVE_MAX:
        return WAVE_MAX, True
    return int(value), False


##################################################################
# wave_line / wave_signed: 32bit 值与 8 位 16 进制文本互转
##################################################################
def wave_line(value):
    """有符号 32bit 值 → 8 位大写 16 进制（无 0x 前缀，负数按补码）"""
    return '%08X' % (int(value) & WAVE_MASK)


def wave_signed(value):
    """32bit 位型 → 有符号十进制"""
    value = int(value) & WAVE_MASK
    return value - 0x100000000 if value & 0x80000000 else value


##################################################################
# iter_wave_points / generate_wave / write_wave_file
##################################################################
def iter_wave_points(code, count, amp, offset, on_progress=None, cancel=None):
    """逐点生成 (值, 是否截断)；cancel 置位时抛 WaveCancelled"""
    for n in range(count):
        if cancel is not None and cancel.is_set():
            raise WaveCancelled()
        yield eval_wave_point(code, n, count, amp, offset)
        if on_progress is not None and (n % 4096 == 0 or n == count - 1):
            on_progress(n + 1, count)


def generate_wave(expr, count, amp, offset, on_progress=None, cancel=None):
    """生成整张波表，返回 (值列表, 截断点数)；取消时抛 WaveCancelled"""
    code = compile_wave_expr(expr)
    values = []
    truncated = 0
    for value, cut in iter_wave_points(code, count, amp, offset, on_progress, cancel):
        values.append(value)
        if cut:
            truncated += 1
    return values, truncated


def write_wave_file(path, expr, count, amp, offset, on_progress=None, cancel=None):
    """
    f(n) 逐点计算并写入文本文件（每 4096 行写入并 flush）。
    返回 (写入点数, 截断点数)；取消/出错时抛异常，半成品文件由调用方删除。
    """
    code = compile_wave_expr(expr)
    truncated = 0
    written = 0
    buffer = []
    with open(path, 'w', encoding='utf-8', newline='\n') as handle:
        for value, cut in iter_wave_points(code, count, amp, offset, None, cancel):
            buffer.append(wave_line(value))
            written += 1
            if cut:
                truncated += 1
            if len(buffer) >= 4096:
                handle.write('\n'.join(buffer) + '\n')
                handle.flush()
                buffer = []
            if on_progress is not None and (written % 4096 == 0 or written == count):
                on_progress(written, count)
        if buffer:
            handle.write('\n'.join(buffer) + '\n')
            handle.flush()
    return written, truncated


##################################################################
# read_text_file / parse_wave_text / parse_wave_file
##################################################################
def read_text_file(path):
    """按 UTF-8(含 BOM) 读取，失败回退 GBK（Windows 记事本常见编码）"""
    with open(path, 'rb') as handle:
        raw = handle.read()
    for codec in ('utf-8-sig', 'gbk'):
        try:
            return raw.decode(codec)
        except UnicodeDecodeError:
            continue
    raise OSError('无法识别文件编码，请把文件保存为 UTF-8')


def _wave_value(token, line_no):
    """解析单个数据：纯 0-9A-F（可带 0x）为 16 进制，带负号为 10 进制"""
    text = token.replace('_', '').replace(' ', '')
    sign = 1
    if text[:1] in ('+', '-'):
        sign = -1 if text[0] == '-' else 1
        text = text[1:]
    if not text:
        raise WaveError('数值为空', line_no)
    hex_mode = text[:2].lower() == '0x'
    if hex_mode:
        text = text[2:]
    if not text:
        raise WaveError('数值为空', line_no)

    if hex_mode or sign > 0:
        if not all(ch in '0123456789abcdefABCDEF' for ch in text):
            raise WaveError('"%s" 不是合法的 16 进制数' % token.strip(), line_no)
        if len(text) > 8:
            raise WaveError('"%s" 超过 8 位 16 进制（最大 FFFFFFFF）'
                            % token.strip(), line_no)
        value = int(text, 16)
    else:
        if not text.isdigit():
            raise WaveError('"%s" 无法解析（负号后请写 10 进制整数）'
                            % token.strip(), line_no)
        value = int(text, 10)
        if value > -WAVE_MIN:
            raise WaveError('"%s" 小于 -2147483648' % token.strip(), line_no)
    return (sign * value) & WAVE_MASK


def parse_wave_text(text):
    """
    解析数据文件，返回 32bit 位型列表（写入时按小端发送）。
    空行忽略；# 或 ; 起到行尾为注释。
    """
    values = []
    for line_no, raw_line in enumerate(text.splitlines(), 1):
        line = raw_line
        for mark in ('#', ';'):
            pos = line.find(mark)
            if pos >= 0:
                line = line[:pos]
        token = line.strip()
        if token:
            values.append(_wave_value(token, line_no))
    return values


def parse_wave_file(path):
    return parse_wave_text(read_text_file(path))


##################################################################
# parse_wave_int / wave_profile: 对话框输入解析与波形预览采样
##################################################################
def parse_wave_int(text, name, default=None):
    """解析整数输入：支持十进制与 0x 前缀 16 进制，允许负号"""
    raw = (text or '').strip().replace('_', '').replace(' ', '')
    if not raw:
        if default is not None:
            return default
        raise WaveError('%s不能为空' % name)
    sign = -1 if raw[0] == '-' else 1
    body = raw[1:] if raw[0] in '+-' else raw
    if not body:
        raise WaveError('%s无法解析: %s' % (name, raw))
    try:
        value = int(body, 16) if body[:2].lower() == '0x' else int(body, 10)
    except ValueError:
        raise WaveError('%s无法解析: %s（可用十进制或 0x 前缀）' % (name, raw))
    return sign * value


def wave_profile(code, count, amp, offset, columns, per_column=8):
    """把 count 个点压缩成 columns 列，返回每列的 (最小值, 最大值)

    每列最多取 per_column 个采样点（必含列首与列尾），大表也能快速画出轮廓。
    """
    count   = max(1, int(count))
    columns = max(1, int(columns))
    profile = []
    for col in range(columns):
        start = int(col * count / float(columns))
        end   = int((col + 1) * count / float(columns)) - 1
        if end < start:
            end = start
        if end > count - 1:
            end = count - 1
        step  = max(1, (end - start + 1) // max(1, per_column))
        picks = list(range(start, end + 1, step))
        if picks[-1] != end:
            picks.append(end)
        low = high = None
        for index in picks:
            value, _cut = eval_wave_point(code, index, count, amp, offset)
            if low is None or value < low:
                low = value
            if high is None or value > high:
                high = value
        profile.append((low, high))
    return profile

def wave_profile_values(values, columns, per_column=8):
    """把已有的 32bit 数据序列压缩成 columns 列，返回每列的 (最小值, 最大值)

    values 可以是 32bit 位型（文件里读到的）或已是有符号整数；
    每列最多取 per_column 个采样点（必含列首与列尾），大文件也能快速画轮廓。
    """
    total   = len(values)
    columns = max(1, int(columns))
    if total <= 0:
        return []
    profile = []
    for col in range(columns):
        start = int(col * total / float(columns))
        end   = int((col + 1) * total / float(columns)) - 1
        if end < start:
            end = start
        if end > total - 1:
            end = total - 1
        step  = max(1, (end - start + 1) // max(1, per_column))
        picks = list(range(start, end + 1, step))
        if picks[-1] != end:
            picks.append(end)
        low = high = None
        for index in picks:
            value = wave_signed(values[index])
            if low is None or value < low:
                low = value
            if high is None or value > high:
                high = value
        profile.append((low, high))
    return profile


##################################################################
# draw_wave_canvas: 在 Canvas 上画波形包络（波表对话框与文件查看共用）
##################################################################
def draw_wave_canvas(canvas, low, high, count, sampler, hint=''):
    """
    画波形轮廓：横轴 n = 0 ~ count-1，纵轴按 low ~ high 自适应缩放。

    sampler(列数) 返回 [(每列最小值, 每列最大值), ...]；为 None 时只显示 hint。
    low/high 为纵轴范围（有符号值），count 为总点数。
    """
    canvas.delete('all')
    width  = canvas.winfo_width()
    height = canvas.winfo_height()
    if width < 120:
        width = 620
    if height < 60:
        height = max(160, canvas.winfo_reqheight())
    left, right, top, bottom = 74, width - 14, 12, height - 22

    if sampler is None:
        canvas.create_text(left + 2, top + 2, anchor='nw', fill='#999999',
                           font=('Segoe UI', 9),
                           text=hint or '没有可显示的数据')
        return

    shown_low, shown_high = low, high
    if high == low:                     # 常量波：放到画布中央
        low, high = low - 1, high + 1
    span = high - low
    if span <= 0:
        span = 1

    def y_of(value):
        return bottom - (value - low) * (bottom - top) / float(span)

    for frac in (0.0, 0.25, 0.5, 0.75, 1.0):
        y = top + (bottom - top) * frac
        canvas.create_line(left, y, right, y, fill='#EEEEEE')
    if low < 0 < high:
        canvas.create_line(left, y_of(0), right, y_of(0), fill='#BDBDBD',
                           dash=(3, 3))
    canvas.create_line(left, top, left, bottom, fill='#DDDDDD')
    try:
        columns = sampler(max(1, right - left))
    except WaveError as exc:
        canvas.create_text(left + 2, top + 2, anchor='nw', fill='#999999',
                           font=('Segoe UI', 9), text=str(exc))
        return
    for index, (col_low, col_high) in enumerate(columns):
        x = left + index
        if x > right:
            break
        y1, y2 = y_of(col_high), y_of(col_low)
        if y2 - y1 < 1.0:
            y2 = y1 + 1.0
        canvas.create_line(x, y1, x, y2, fill='#1565C0')
    canvas.create_text(left - 6, top + 4, anchor='ne', fill='#666666',
                       font=('Consolas', 8), text=wave_line(shown_high))
    canvas.create_text(left - 6, bottom - 4, anchor='se', fill='#666666',
                       font=('Consolas', 8), text=wave_line(shown_low))
    canvas.create_text(left, height - 6, anchor='sw', fill='#666666',
                       font=('Consolas', 8), text='n=0')
    if count > 0:
        canvas.create_text(right, height - 6, anchor='se', fill='#666666',
                           font=('Consolas', 8), text='n=%d' % (count - 1))


##################################################################
# DebugBus: 在 UartBusInterface 之上增加超时保护与定长读写
##################################################################
class DebugBus(UartBusInterface):
    ##################################################################
    # Construction
    ##################################################################
    def __init__(self, port, baud, timeout=0.5):
        UartBusInterface.__init__(self, port, baud)
        self.timeout = timeout
        self._lock   = threading.Lock()

    ##################################################################
    # is_open: 串口是否已打开
    ##################################################################
    @property
    def is_open(self):
        return self.uart is not None and getattr(self.uart, 'is_open', False)

    ##################################################################
    # connect: 只打开串口（自检由 self_check 单独完成）
    ##################################################################
    def connect(self):
        if self.is_open:
            return
        try:
            self.uart = serial.Serial(port=self.interface,
                                      baudrate=self.baud,
                                      bytesize=serial.EIGHTBITS,
                                      parity=serial.PARITY_NONE,
                                      stopbits=serial.STOPBITS_ONE,
                                      timeout=self.timeout)
        except Exception as exc:
            self.uart = None
            raise IOError(self._open_error_text(exc))

    ##################################################################
    # _open_error_text: 把 pyserial 的英文异常翻译成可读中文
    ##################################################################
    def _open_error_text(self, exc):
        busy    = '串口 %s 被其他程序占用或无访问权限，请关闭占用它的程序后重试' % self.interface
        missing = '串口 %s 不存在，请检查 USB 线是否插好并重新扫描端口' % self.interface
        node, depth = exc, 0
        while node is not None and depth < 8:
            if isinstance(node, PermissionError):
                return busy
            if isinstance(node, FileNotFoundError):
                return missing
            node, depth = getattr(node, '__context__', None), depth + 1
        text = str(exc)
        low  = text.lower()
        if 'permissionerror' in low or 'errno 13' in low or \
                '拒绝访问' in text or 'access is denied' in low:
            return busy
        if 'filenotfounderror' in low or 'errno 2' in low or 'could not find' in low or \
                '找不到' in text:
            return missing
        return '打开串口 %s 失败: %s' % (self.interface, text)

    ##################################################################
    # close: 关闭串口
    ##################################################################
    def close(self):
        with self._lock:
            uart, self.uart = self.uart, None
        if uart is not None:
            try:
                uart.close()
            except Exception:
                pass

    ##################################################################
    # _flush_input: 清空接收缓冲，保证事务对齐
    ##################################################################
    def _flush_input(self):
        try:
            self.uart.reset_input_buffer()
        except Exception:
            pass

    ##################################################################
    # read_word: 变长读取 (1/2/4 字节)，数据小端拼接
    ##################################################################
    def read_word(self, addr, size):
        if not self.is_open:
            raise IOError('串口未打开')
        with self._lock:
            self._flush_input()
            try:
                data = self.read(addr & 0xFFFFFFFF, size, addr_incr=False)
            except TypeError:
                raise TimeoutError('读取超时: %d 字节未能全部返回，请检查波特率/接线' % size)
            except serial.SerialTimeoutException:
                raise TimeoutError('读取超时: %d 字节未能全部返回，请检查波特率/接线' % size)
            except serial.SerialException as exc:
                raise IOError('串口读取出错: %s' % exc)
            finally:
                self._flush_input()
        raw = bytes(data)
        if len(raw) != size:
            raise TimeoutError('读取超时: 期望 %d 字节，实际 %d 字节' % (size, len(raw)))
        return int.from_bytes(raw, 'little')

    ##################################################################
    # write_word: 变长写入 (1/2/4 字节)，数据小端发送
    ##################################################################
    def write_word(self, addr, value, size):
        if not self.is_open:
            raise IOError('串口未打开')
        payload = int(value).to_bytes(size, 'little')
        with self._lock:
            self._flush_input()
            try:
                self.write(addr & 0xFFFFFFFF, payload, size, addr_incr=False)
            except serial.SerialException as exc:
                raise IOError('串口写入出错: %s' % exc)
        return payload

    ##################################################################
    # write_chunk: 一次写一块（1..255 字节），地址不自增
    ##################################################################
    def write_chunk(self, addr, payload):
        if not self.is_open:
            raise IOError('串口未打开')
        data = bytes(payload)
        if len(data) < 1 or len(data) > self.MAX_SIZE:
            raise ValueError('单次写入长度必须在 1..%d 字节' % self.MAX_SIZE)
        with self._lock:
            self._flush_input()
            try:
                self.write(addr & 0xFFFFFFFF, data, len(data), addr_incr=False,
                           max_block_size=self.MAX_SIZE)
            except serial.SerialException as exc:
                raise IOError('串口写入出错: %s' % exc)
        return len(data)

    ##################################################################
    # read_chunk: 一次读一块（1..255 字节），返回原始字节（小端数据）
    ##################################################################
    def read_chunk(self, addr, size):
        if not self.is_open:
            raise IOError('串口未打开')
        if size < 1 or size > self.MAX_SIZE:
            raise ValueError('单次读取长度必须在 1..%d 字节' % self.MAX_SIZE)
        with self._lock:
            self._flush_input()
            try:
                data = self.read(addr & 0xFFFFFFFF, size, addr_incr=False,
                                 max_block_size=self.MAX_SIZE)
            except TypeError:
                raise TimeoutError('读取超时: %d 字节未能全部返回，请检查波特率/接线' % size)
            except serial.SerialTimeoutException:
                raise TimeoutError('读取超时: %d 字节未能全部返回，请检查波特率/接线' % size)
            except serial.SerialException as exc:
                raise IOError('串口读取出错: %s' % exc)
            finally:
                self._flush_input()
        raw = bytes(data)
        if len(raw) != size:
            raise TimeoutError('读取超时: 期望 %d 字节，实际 %d 字节' % (size, len(raw)))
        return raw

    ##################################################################
    # self_check: 读取状态寄存器并判断 0xCAFE 标识
    ##################################################################
    def self_check(self, addr=None):
        """读取状态寄存器（默认 STS_ADDR=0xF0000004），返回 32bit 值"""
        return self.read_word(self.STS_ADDR if addr is None else (addr & 0xFFFFFFFF), 4)


##################################################################
# DebugApp: 图形界面
##################################################################
class DebugApp:
    COLOR_IDLE = '#606060'
    COLOR_BUSY = '#b06000'
    COLOR_OK   = '#137333'
    COLOR_ERR  = '#c5221f'

    ##################################################################
    # Construction
    ##################################################################
    def __init__(self, root, port='', baud=1000000, timeout=0.5):
        self.root     = root
        self.bus      = None
        self.busy     = False
        self.timeout  = timeout
        self._last_values     = {}
        self._action_widgets  = []
        self._queue           = queue.Queue()
        self._prev_addr_radix = 'hex'
        self._prev_data_radix = 'hex'

        root.title('UART2AXI 调试工具 - 32bit 寄存器读写')
        root.minsize(910, 600)

        self.port_var      = tk.StringVar(value=port)
        self.baud_var      = tk.StringVar(value=str(baud))
        self.addr_radix    = tk.StringVar(value='hex')
        self.data_radix    = tk.StringVar(value='hex')
        self.width_var     = tk.IntVar(value=4)
        self.addr_var      = tk.StringVar(value='0x00000000')
        self.write_var     = tk.StringVar(value='0x00000000')
        self.read_var      = tk.StringVar(value='')
        self.gpio_addr_var = tk.StringVar(value='0x%08X' % GPIO_ADDR)
        self.sts_addr_var  = tk.StringVar(value='0x%08X' % STS_ADDR)
        self.status_var    = tk.StringVar(value='● 未连接')

        self._build_ui()
        self._refresh_ports()
        self.root.protocol('WM_DELETE_WINDOW', self.on_close)

    ##################################################################
    # UI construction
    ##################################################################
    def _build_ui(self):
        main = ttk.Frame(self.root, padding=8)
        main.grid(row=0, column=0, sticky='nsew')
        self.root.rowconfigure(0, weight=1)
        self.root.columnconfigure(0, weight=1)
        main.columnconfigure(0, weight=1, uniform='panel')
        main.columnconfigure(1, weight=1, uniform='panel')
        main.rowconfigure(2, weight=1)

        self._build_serial_frame(main).grid(row=0, column=0, columnspan=2,
                                            sticky='ew', pady=(0, 6))
        self._build_access_frame(main).grid(row=1, column=0, sticky='nsew',
                                            padx=(0, 3), pady=(0, 6))
        self._build_file_frame(main).grid(row=1, column=1, sticky='nsew',
                                          padx=(3, 0), pady=(0, 6))
        self._build_log_frame(main).grid(row=2, column=0, columnspan=2,
                                         sticky='nsew')

    ##################################################################
    # 串口设置
    ##################################################################
    def _build_serial_frame(self, parent):
        frame = ttk.LabelFrame(parent, text=' 串口设置 ', padding=8)
        frame.columnconfigure(1, weight=1)

        ttk.Label(frame, text='COM 口').grid(row=0, column=0, sticky='w')
        self.port_combo = ttk.Combobox(frame, textvariable=self.port_var, width=26)
        self.port_combo.grid(row=0, column=1, sticky='ew', padx=(6, 6))
        self.refresh_btn = ttk.Button(frame, text='刷新', width=6, command=self._refresh_ports)
        self.refresh_btn.grid(row=0, column=2, padx=(0, 14))

        ttk.Label(frame, text='波特率').grid(row=0, column=3, sticky='w')
        self.baud_combo = ttk.Combobox(frame, textvariable=self.baud_var,
                                       values=BAUD_RATES, width=12)
        self.baud_combo.grid(row=0, column=4, sticky='w', padx=(6, 14))

        self.open_btn = ttk.Button(frame, text='打开串口', width=10,
                                   command=self.on_toggle_serial)
        self.open_btn.grid(row=0, column=5, sticky='e')

        self.status_label = tk.Label(frame, textvariable=self.status_var, anchor='w',
                                    fg=self.COLOR_IDLE,
                                    font=('Microsoft YaHei UI', 9, 'bold'))
        self.status_label.grid(row=1, column=0, columnspan=6, sticky='w', pady=(8, 0))

        self._action_widgets += [self.refresh_btn, self.open_btn]
        return frame


    ##################################################################
    # 寄存器访问
    ##################################################################
    def _build_access_frame(self, parent):
        frame = ttk.LabelFrame(parent, text=' 寄存器访问 ', padding=8)
        frame.columnconfigure(1, weight=1)

        radix_box = ttk.Frame(frame)
        radix_box.grid(row=0, column=0, columnspan=4, sticky='w')
        ttk.Label(radix_box, text='地址进制：').pack(side='left')
        ttk.Radiobutton(radix_box, text='16 进制', value='hex', variable=self.addr_radix,
                        command=self._on_addr_radix).pack(side='left')
        ttk.Radiobutton(radix_box, text='10 进制', value='dec', variable=self.addr_radix,
                        command=self._on_addr_radix).pack(side='left', padx=(0, 20))
        ttk.Label(radix_box, text='数据进制：').pack(side='left')
        ttk.Radiobutton(radix_box, text='16 进制', value='hex', variable=self.data_radix,
                        command=self._on_data_radix).pack(side='left')
        ttk.Radiobutton(radix_box, text='10 进制', value='dec', variable=self.data_radix,
                        command=self._on_data_radix).pack(side='left')

        width_box = ttk.Frame(frame)
        width_box.grid(row=1, column=0, columnspan=4, sticky='w', pady=(8, 0))
        ttk.Label(width_box, text='访问宽度：').pack(side='left')
        for width in WIDTHS:
            ttk.Radiobutton(width_box, text=WIDTH_TEXT[width], value=width,
                            variable=self.width_var,
                            command=self._on_width).pack(side='left')
        ttk.Label(width_box, text='（地址按宽度向下对齐）',
                  foreground='#666666').pack(side='left', padx=(12, 0))

        ttk.Label(frame, text='地址').grid(row=2, column=0, sticky='w', pady=(8, 0))
        self.addr_entry = ttk.Entry(frame, textvariable=self.addr_var,
                                    font=('Consolas', 10))
        self.addr_entry.grid(row=2, column=1, sticky='ew', padx=(6, 12), pady=(8, 0))
        ttk.Label(frame, text='(32 bit)').grid(row=2, column=2, sticky='w', pady=(8, 0))

        ttk.Label(frame, text='写入数据').grid(row=3, column=0, sticky='w', pady=(8, 0))
        self.write_entry = ttk.Entry(frame, textvariable=self.write_var,
                                     font=('Consolas', 10))
        self.write_entry.grid(row=3, column=1, sticky='ew', padx=(6, 12), pady=(8, 0))
        self.write_btn = ttk.Button(frame, text='写 入', width=10, command=self.on_write)
        self.write_btn.grid(row=3, column=2, sticky='w', pady=(8, 0))

        ttk.Label(frame, text='读取数据').grid(row=4, column=0, sticky='w', pady=(8, 0))
        self.read_entry = ttk.Entry(frame, textvariable=self.read_var, state='readonly',
                                    font=('Consolas', 10))
        self.read_entry.grid(row=4, column=1, sticky='ew', padx=(6, 12), pady=(8, 0))

        self.read_btn = ttk.Button(frame, text='读 取', width=10, command=self.on_read)
        self.read_btn.grid(row=4, column=2, sticky='w', pady=(8, 0))

        # 预设地址（可编辑）：GPIO 快捷填入 / 目标检测
        preset_box = ttk.Frame(frame)
        preset_box.grid(row=5, column=0, columnspan=4, sticky='w', pady=(8, 0))

        ttk.Label(preset_box, text='GPIO 地址').pack(side='left')
        self.gpio_addr_entry = ttk.Entry(preset_box, textvariable=self.gpio_addr_var,
                                         width=10, font=('Consolas', 10))
        self.gpio_addr_entry.pack(side='left', padx=(6, 6))
        self.gpio_fill_btn = ttk.Button(preset_box, text='填 入', width=6,
                                        command=self.on_gpio_fill)
        self.gpio_fill_btn.pack(side='left', padx=(0, 12))

        ttk.Label(preset_box, text='检测地址').pack(side='left')
        self.sts_addr_entry = ttk.Entry(preset_box, textvariable=self.sts_addr_var,
                                        width=10, font=('Consolas', 10))
        self.sts_addr_entry.pack(side='left', padx=(6, 6))
        self.probe_btn = ttk.Button(preset_box, text='检测目标', width=8,
                                    command=self.on_self_check)
        self.probe_btn.pack(side='left')

        self.test_hint = ttk.Label(frame, text='（预设地址可改，均按 4 字节）',
                                   foreground='#666666')
        self.test_hint.grid(row=6, column=0, columnspan=4, sticky='w', pady=(6, 0))

        self._action_widgets += [self.write_btn, self.read_btn,
                                 self.gpio_fill_btn, self.probe_btn]
        self.addr_entry.bind('<Return>', lambda event: self.on_read())
        self.write_entry.bind('<Return>', lambda event: self.on_write())
        self.gpio_addr_entry.bind('<Return>', lambda event: self.on_gpio_fill())
        self.sts_addr_entry.bind('<Return>', lambda event: self.on_self_check())
        return frame

    ##################################################################
    # 通信日志
    ##################################################################
    def _build_log_frame(self, parent):
        frame = ttk.LabelFrame(parent, text=' 通信日志 ', padding=6)
        frame.columnconfigure(0, weight=1)
        frame.rowconfigure(0, weight=1)

        self.log_text = tk.Text(frame, height=12, width=90, wrap='none',
                                font=('Consolas', 9), state='disabled',
                                background='#fbfbfb', relief='solid', borderwidth=1)
        self.log_text.grid(row=0, column=0, sticky='nsew')
        scroll = ttk.Scrollbar(frame, orient='vertical', command=self.log_text.yview)
        scroll.grid(row=0, column=1, sticky='ns')
        self.log_text.configure(yscrollcommand=scroll.set)

        self.log_text.tag_configure('info', foreground='#333333')
        self.log_text.tag_configure('tx',   foreground='#8a5a00')
        self.log_text.tag_configure('rx',   foreground='#135c13')
        self.log_text.tag_configure('ok',   foreground='#137333')
        self.log_text.tag_configure('err',  foreground='#c5221f')
        return frame

    ##################################################################
    # 日志 / 状态栏
    ##################################################################
    def log(self, message, tag='info'):
        stamp = time.strftime('%H:%M:%S') + '.%03d' % (int(time.time() * 1000) % 1000)
        self.log_text.configure(state='normal')
        self.log_text.insert('end', '%s  %s\n' % (stamp, message), tag)
        self.log_text.see('end')
        self.log_text.configure(state='disabled')

    def _set_status(self, text, color):
        self.status_var.set(text)
        self.status_label.configure(fg=color)

    ##################################################################
    # 串口枚举
    ##################################################################
    def _refresh_ports(self):
        entries = []
        try:
            for info in list_ports.comports():
                desc = (info.description or '').strip()
                if desc and desc != 'n/a':
                    entries.append('%s - %s' % (info.device, desc))
                else:
                    entries.append(info.device)
        except Exception as exc:
            self.log('枚举串口失败: %s' % exc, 'err')

        self.port_combo.configure(values=entries)
        if not self.port_var.get().strip() and entries:
            self.port_var.set(entries[0])
        self.log('发现 %d 个串口: %s' % (len(entries), ', '.join(entries) if entries else '无'), 'info')

    def _selected_port(self):
        text = (self.port_var.get() or '').strip()
        if ' - ' in text:
            text = text.split(' - ', 1)[0].strip()
        return text

    ##################################################################
    # 打开 / 关闭串口
    ##################################################################
    def on_toggle_serial(self):
        if self.bus is not None and self.bus.is_open:
            self._close_serial()
            return

        port = self._selected_port()
        if not port:
            self._warn('请选择或输入 COM 口')
            return
        try:
            baud = int((self.baud_var.get() or '').strip())
            if baud <= 0:
                raise ValueError
        except ValueError:
            self._warn('波特率必须是正整数')
            return

        self.bus = DebugBus(port, baud, timeout=self.timeout)
        self.log('打开串口 %s @ %d 8N1，读超时 %.2fs' % (port, baud, self.timeout), 'info')
        self._run_async(self._open_and_check, self._open_done)

    def _open_and_check(self):
        self.bus.connect()
        try:
            return self.bus.self_check(), None
        except Exception as exc:
            return None, exc

    def _open_done(self, result, error):
        if error is not None:
            self.bus = None
            self.log('打开串口失败: %s' % error, 'err')
            self._set_status('● 未连接 · %s' % error, self.COLOR_ERR)
            return

        value, check_error = result
        self.open_btn.configure(text='关闭串口')
        self._set_serial_enabled(False)
        self.log('串口已打开: %s @ %d' % (self.bus.interface, self.bus.baud), 'ok')
        self._report_self_check(value, check_error)

    def _close_serial(self):
        if self.bus is not None:
            self.bus.close()
        self.bus = None
        self.open_btn.configure(text='打开串口')
        self._set_serial_enabled(True)
        self._set_status('● 未连接', self.COLOR_IDLE)
        self.log('串口已关闭', 'info')

    def _set_serial_enabled(self, enabled):
        state = 'normal' if enabled else 'disabled'
        self.port_combo.configure(state=state)
        self.baud_combo.configure(state=state)

    ##################################################################
    # 忙碌状态 / 后台线程
    ##################################################################
    def _set_busy(self, busy):
        self.busy = busy
        state = 'disabled' if busy else 'normal'
        for widget in self._action_widgets:
            widget.configure(state=state)

    def _run_async(self, work, done):
        if self.busy:
            return
        self._set_busy(True)
        self._set_status('● 通信中...', self.COLOR_BUSY)

        def runner():
            try:
                result = work()
            except Exception as exc:
                self._queue.put((done, None, exc))
            else:
                self._queue.put((done, result, None))

        threading.Thread(target=runner, daemon=True).start()
        self.root.after(20, self._poll_queue)

    def _poll_queue(self):
        try:
            done, result, error = self._queue.get_nowait()
        except queue.Empty:
            self.root.after(20, self._poll_queue)
            return
        self._set_busy(False)
        done(result, error)

    def on_self_check(self):
        if not self._require_bus():
            return
        try:
            addr = self._parse_addr(self.sts_addr_var.get(), '检测地址')
        except ValueError as exc:
            self._warn(str(exc))
            return

        radix = self.addr_radix.get()
        addr  = self._align_addr(addr, 4)
        self.sts_addr_var.set(self._fmt(addr, radix, 8))
        frame = self._read_frame(addr, 4)
        self.log('TX: %s   [自检 %s, 4 字节]'
                 % (self._hex_bytes(frame), self._fmt(addr, radix, 8)), 'tx')
        self._run_async(lambda: self.bus.self_check(addr), self._report_self_check)

    def _report_self_check(self, value, error):
        if error is not None:
            self.log('自检失败: %s' % error, 'err')
            self._set_status('● 已连接 · 目标无响应（检查波特率/接线）', self.COLOR_ERR)
            return

        text = '%s = 0x%08X' % (self.sts_addr_var.get(), value)
        if (value & 0xFFFF0000) == STS_MAGIC:
            self.log('自检通过: %s（期望 0xCAFE****）' % text, 'ok')
            self._set_status('● 已连接 · 自检通过 (%s)' % text, self.COLOR_OK)
        else:
            self.log('自检失败: 期望 0xCAFE****，实际 %s' % text, 'err')
            self._set_status('● 已连接 · 自检失败 (%s)' % text, self.COLOR_ERR)

    def _require_bus(self):
        if self.bus is not None and self.bus.is_open:
            return True
        self._warn('请先打开串口')
        return False

    def _warn(self, message):
        self.log(message, 'err')
        prefix = '● 已连接 · ' if (self.bus is not None and self.bus.is_open) else '● 未连接 · '
        self._set_status(prefix + message, self.COLOR_ERR)

    def _fail(self, error):
        self.log('操作失败: %s' % error, 'err')
        prefix = '● 已连接 · ' if (self.bus is not None and self.bus.is_open) else '● 未连接 · '
        self._set_status(prefix + str(error), self.COLOR_ERR)

    ##################################################################
    # 数值解析 / 格式化
    ##################################################################
    @staticmethod
    def _parse_int(text, radix):
        raw = (text or '').strip().replace('_', '').replace(' ', '')
        if not raw:
            raise ValueError('数值不能为空')
        try:
            if radix == 'hex':
                if raw.lower().startswith('0x'):
                    raw = raw[2:]
                if not raw:
                    raise ValueError
                return int(raw, 16)
            return int(raw, 10)
        except ValueError:
            raise ValueError('无法解析 "%s"（当前为%s模式）'
                             % (text, '16 进制' if radix == 'hex' else '10 进制'))

    def _parse_addr(self, text, name='地址'):
        value = self._parse_int(text, self.addr_radix.get())
        if value < 0 or value > 0xFFFFFFFF:
            raise ValueError('%s超出 32bit 范围（0 ~ 0xFFFFFFFF）' % name)
        return value

    def _parse_data(self, text, name='数据'):
        width = self.width_var.get()
        value = self._parse_int(text, self.data_radix.get())
        limit = (1 << (8 * width)) - 1
        if value < 0 or value > limit:
            raise ValueError('%s超出 %d 字节范围（0 ~ 0x%X）' % (name, width, limit))
        return value

    @staticmethod
    def _align_addr(addr, width):
        """
        按访问宽度把地址向下对齐（不报错，只修正）。
        4 字节访问低 2 位清零（0x1 -> 0x0），2 字节访问低 1 位清零，1 字节地址不变。
        """
        return (addr & ~(width - 1)) & 0xFFFFFFFF

    def _aligned_addr(self, addr, width, label, var=None):
        """地址未按宽度对齐时向下对齐，并在日志中说明实际使用的地址"""
        aligned = self._align_addr(addr, width)
        if aligned != addr:
            self.log('%s地址对齐: %s -> %s（%d 字节访问忽略低 %d 位）' % (
                label,
                self._fmt(addr, self.addr_radix.get(), 8),
                self._fmt(aligned, self.addr_radix.get(), 8),
                width, (width - 1).bit_length()), 'info')
            if var is not None:
                var.set(self._fmt(aligned, self.addr_radix.get(), 8))
        return aligned

    @staticmethod
    def _fmt(value, radix, digits):
        if radix == 'hex':
            return '0x%0*X' % (digits, value)
        return str(value)

    @staticmethod
    def _hex_bytes(data):
        return ' '.join('%02X' % byte for byte in bytearray(data))

    @staticmethod
    def _read_frame(addr, size):
        return bytes([CMD_READ, size,
                      (addr >> 24) & 0xFF, (addr >> 16) & 0xFF,
                      (addr >> 8) & 0xFF, addr & 0xFF])

    @staticmethod
    def _write_frame(addr, value, size):
        return bytes([CMD_WRITE, size,
                      (addr >> 24) & 0xFF, (addr >> 16) & 0xFF,
                      (addr >> 8) & 0xFF, addr & 0xFF]) + int(value).to_bytes(size, 'little')

    ##################################################################
    # 进制 / 宽度切换
    ##################################################################
    def _on_addr_radix(self):
        new_radix = self.addr_radix.get()
        old_radix = self._prev_addr_radix
        if new_radix == old_radix:
            return
        self._prev_addr_radix = new_radix
        for var in (self.addr_var, self.gpio_addr_var, self.sts_addr_var):
            try:
                value = self._parse_int(var.get(), old_radix)
            except ValueError:
                continue
            if 0 <= value <= 0xFFFFFFFF:
                var.set(self._fmt(value, new_radix, 8))

    def _on_data_radix(self):
        new_radix = self.data_radix.get()
        old_radix = self._prev_data_radix
        if new_radix == old_radix:
            return
        self._prev_data_radix = new_radix
        width = self.width_var.get()
        limit = (1 << (8 * width)) - 1
        for var in (self.write_var,):
            try:
                value = self._parse_int(var.get(), old_radix)
            except ValueError:
                continue
            if 0 <= value <= limit:
                var.set(self._fmt(value, new_radix, width * 2))
        for var, value in list(self._last_values.values()):
            var.set(self._fmt(value, new_radix, width * 2))

    def _on_width(self):
        width = self.width_var.get()
        limit = (1 << (8 * width)) - 1
        radix = self.data_radix.get()
        for var in (self.write_var,):
            try:
                value = self._parse_int(var.get(), radix)
            except ValueError:
                continue
            if 0 <= value <= limit:
                var.set(self._fmt(value, radix, width * 2))
        for var, value in list(self._last_values.values()):
            if 0 <= value <= limit:
                var.set(self._fmt(value, radix, width * 2))

    ##################################################################
    # 按钮回调
    ##################################################################
    def on_write(self):
        self._do_write(self.addr_var, self.write_var, '寄存器')

    def on_read(self):
        self._do_read(self.addr_var, self.read_var, '寄存器', 'reg')

    def on_gpio_fill(self):
        """快捷填入：把预设 GPIO 地址写入上方地址栏并切换为 4 字节访问（不自动发起访问）"""
        try:
            addr = self._parse_addr(self.gpio_addr_var.get(), 'GPIO 预设地址')
        except ValueError as exc:
            self._warn(str(exc))
            return

        radix = self.addr_radix.get()
        addr  = self._align_addr(addr, 4)
        self.gpio_addr_var.set(self._fmt(addr, radix, 8))
        self.width_var.set(4)
        self._on_width()                       # 数据字段按 4 字节重新格式化
        self.addr_var.set(self._fmt(addr, radix, 8))
        self.log('已填入 GPIO 地址 %s（4 字节访问），按「读 取」/「写 入」执行'
                 % self._fmt(addr, radix, 8), 'info')

    # ---- 读写操作 ----
    def _do_write(self, addr_var, data_var, label):
        if not self._require_bus():
            return
        width = self.width_var.get()
        try:
            addr  = self._parse_addr(addr_var.get(), label + '地址')
            value = self._parse_data(data_var.get(), label + '数据')
        except ValueError as exc:
            self._warn(str(exc))
            return

        addr = self._aligned_addr(addr, width, label, addr_var)

        frame = self._write_frame(addr, value, width)
        self.log('TX: %s   [写%s %s @ %s, %d 字节]' % (
            self._hex_bytes(frame), label,
            self._fmt(value, self.data_radix.get(), width * 2),
            self._fmt(addr, self.addr_radix.get(), 8), width), 'tx')

        def work():
            return self.bus.write_word(addr, value, width)

        def done(result, error):
            if error is not None:
                self._fail(error)
                return
            self.log('写入完成: %s @ %s' % (
                self._fmt(value, self.data_radix.get(), width * 2),
                self._fmt(addr, self.addr_radix.get(), 8)), 'ok')
            self._set_status('● 已连接 · 写入完成', self.COLOR_OK)

        self._run_async(work, done)

    def _do_read(self, addr_var, out_var, label, key):
        if not self._require_bus():
            return
        width = self.width_var.get()
        try:
            addr = self._parse_addr(addr_var.get(), label + '地址')
        except ValueError as exc:
            self._warn(str(exc))
            return

        addr = self._aligned_addr(addr, width, label, addr_var)

        frame = self._read_frame(addr, width)
        self.log('TX: %s   [读%s %s, %d 字节]' % (
            self._hex_bytes(frame), label,
            self._fmt(addr, self.addr_radix.get(), 8), width), 'tx')

        def work():
            return self.bus.read_word(addr, width)

        def done(value, error):
            if error is not None:
                self._fail(error)
                return
            self._last_values[key] = (out_var, value)
            out_var.set(self._fmt(value, self.data_radix.get(), width * 2))
            raw = int(value).to_bytes(width, 'little')
            self.log('RX: %s  →  0x%0*X (%d)' % (
                self._hex_bytes(raw), width * 2, value, value), 'rx')
            self._set_status('● 已连接 · 读取完成', self.COLOR_OK)

        self._run_async(work, done)

    # ---- 窗口关闭 / 程序入口 ----
    def on_close(self):
        self.script_cancel.set()
        self.batch_cancel.set()
        if self._wave_cancel is not None:
            self._wave_cancel.set()
        if self.wave_win is not None:
            try:
                self.wave_win.destroy()
            except Exception:
                pass
            self.wave_win = None
        if self.wave_plot_win is not None:
            self._close_wave_plot()
        if self.bus is not None:
            try:
                self.bus.close()
            except Exception:
                pass
            self.bus = None
        self.root.destroy()

    ##################################################################
    # 脚本 / 批量（新增：读取文本文件，逐行执行 4 字节操作）
    ##################################################################
    def _build_file_frame(self, parent):
        frame = ttk.LabelFrame(parent, text=' 脚本 / 批量 ', padding=8)
        frame.columnconfigure(0, weight=1)

        self.script_path_var  = tk.StringVar(value='')
        self.script_count_var = tk.StringVar(value='0 / 0')
        self.script_ops       = []
        self.script_queue     = queue.Queue()
        self.script_cancel    = threading.Event()
        self.script_active    = False
        self._script_started  = 0.0

        self.wave_path_var    = tk.StringVar(value='')
        self.wave_out_var     = tk.StringVar(value='')
        self.batch_addr_var   = tk.StringVar(value='')
        self.batch_count_var  = tk.StringVar(value='')
        self.batch_block_var  = tk.StringVar(value=str(BATCH_BLOCK_WORDS))
        self.batch_mode       = ''
        self.batch_queue      = queue.Queue()
        self.batch_cancel     = threading.Event()
        self.batch_active     = False
        self._batch_started   = 0.0
        self.wave_win            = None
        self.wave_plot_win       = None
        self.wave_plot_canvas    = None
        self.wave_plot_data      = None
        self.wave_plot_path_var  = None
        self.wave_plot_stat_var  = None
        self.wave_plot_stat_label = None
        self._wave_cancel        = None

        self.script_book = ttk.Notebook(frame)
        self.script_book.grid(row=0, column=0, sticky='ew')

        # ---- 脚本页：逐行执行 <地址>:<数据> ----
        page = ttk.Frame(self.script_book, padding=(4, 6))
        page.columnconfigure(1, weight=1)
        self.script_book.add(page, text=' 脚本 ')

        ttk.Label(page, text='脚本').grid(row=0, column=0, sticky='w')
        self.script_path_entry = ttk.Entry(page, textvariable=self.script_path_var,
                                           state='readonly', font=('Consolas', 10),
                                           width=22)
        self.script_path_entry.grid(row=0, column=1, sticky='ew', padx=(6, 0))
        self.script_open_btn = ttk.Button(page, text='打开脚本…', width=11,
                                          command=self.on_load_script)
        self.script_open_btn.grid(row=1, column=0, sticky='w', pady=(8, 0))

        # ---- 批量 / 波表页：连续 N 点 4 字节读写 ----
        page = ttk.Frame(self.script_book, padding=(4, 6))
        page.columnconfigure(1, weight=1)
        self.script_book.add(page, text=' 批量 / 波表 ')

        top = ttk.Frame(page)
        top.grid(row=0, column=0, columnspan=3, sticky='w')
        ttk.Label(top, text='起始地址').pack(side='left')
        self.batch_addr_entry = ttk.Entry(top, textvariable=self.batch_addr_var,
                                          font=('Consolas', 10), width=12)
        self.batch_addr_entry.pack(side='left', padx=(6, 12))
        ttk.Label(top, text='点数').pack(side='left')
        self.batch_count_entry = ttk.Entry(top, textvariable=self.batch_count_var,
                                           font=('Consolas', 10), width=7)
        self.batch_count_entry.pack(side='left', padx=(6, 12))
        ttk.Label(top, text='每块').pack(side='left')
        self.batch_block_entry = ttk.Entry(top, textvariable=self.batch_block_var,
                                           font=('Consolas', 10), width=4)
        self.batch_block_entry.pack(side='left', padx=(6, 0))

        ttk.Label(page, text='数据文件').grid(row=1, column=0, sticky='w', pady=(8, 0))
        self.wave_path_entry = ttk.Entry(page, textvariable=self.wave_path_var,
                                         state='readonly', font=('Consolas', 10),
                                         width=18)
        self.wave_path_entry.grid(row=1, column=1, sticky='ew', padx=(6, 6), pady=(8, 0))
        self.wave_pick_btn = ttk.Button(page, text='选择…', width=8,
                                        command=self.on_pick_wave_file)
        self.wave_pick_btn.grid(row=1, column=2, sticky='e', pady=(8, 0))

        ttk.Label(page, text='输出文件').grid(row=2, column=0, sticky='w', pady=(6, 0))
        self.wave_out_entry = ttk.Entry(page, textvariable=self.wave_out_var,
                                        state='readonly', font=('Consolas', 10),
                                        width=18)
        self.wave_out_entry.grid(row=2, column=1, sticky='ew', padx=(6, 6), pady=(6, 0))
        self.wave_out_btn = ttk.Button(page, text='另存…', width=8,
                                       command=self.on_pick_out_file)
        self.wave_out_btn.grid(row=2, column=2, sticky='e', pady=(6, 0))

        box = ttk.Frame(page)
        box.grid(row=3, column=0, columnspan=3, sticky='w', pady=(8, 0))
        self.wave_gen_btn = ttk.Button(box, text='生成波表…', width=10,
                                       command=self.on_open_wave_gen)
        self.wave_gen_btn.pack(side='left')
        self.view_wave_btn = ttk.Button(box, text='查看波形…', width=9,
                                        command=self.on_show_wave_plot)
        self.view_wave_btn.pack(side='left', padx=(6, 0))
        self.write_wave_btn = ttk.Button(box, text='连续写入', width=9,
                                         command=self.on_write_wave)
        self.write_wave_btn.pack(side='left', padx=(6, 0))
        self.read_wave_btn = ttk.Button(box, text='连续读取', width=9,
                                        command=self.on_read_wave)
        self.read_wave_btn.pack(side='left', padx=(6, 0))

        # ---- 共用进度 / 计数 / 停止 ----
        box = ttk.Frame(frame)
        box.grid(row=1, column=0, sticky='ew', pady=(8, 0))
        self.script_prog = ttk.Progressbar(box, orient='horizontal',
                                           mode='determinate', maximum=1, value=0)
        self.script_prog.pack(side='left', fill='x', expand=True)
        ttk.Label(box, textvariable=self.script_count_var, width=10).pack(
            side='left', padx=(8, 0))
        self.script_stop_btn = ttk.Button(box, text='停 止', width=7, state='disabled',
                                          command=self.on_stop_script)
        self.script_stop_btn.pack(side='left', padx=(8, 0))

        ttk.Label(frame, text='16 进制；写入用「数据文件」，读取存到「输出文件」，每点固定 4 字节；'
                              '「查看波形…」可直接看两者的真实波形',
                  foreground='#666666', wraplength=400, justify='left').grid(
            row=2, column=0, sticky='w', pady=(8, 0))

        self._action_widgets += [self.script_open_btn, self.wave_pick_btn,
                                 self.wave_out_btn, self.wave_gen_btn,
                                 self.view_wave_btn,
                                 self.write_wave_btn, self.read_wave_btn]
        return frame

    ##################################################################
    # 不带时间戳的日志（脚本执行用：开始/结束时间在首尾行显式给出）
    ##################################################################
    def _log_plain(self, message, tag='info'):
        self.log_text.configure(state='normal')
        self.log_text.insert('end', message + '\n', tag)
        self.log_text.see('end')
        self.log_text.configure(state='disabled')

    ##################################################################
    # 打开脚本 → 解析 → 预览（按文件顺序从上到下）
    ##################################################################
    @staticmethod
    def _read_script_file(path):
        """按 UTF-8(含 BOM) 读取，失败回退 GBK（Windows 记事本常见编码）"""
        return read_text_file(path)

    def on_load_script(self):
        if self.script_active or self.batch_active:
            self._warn('脚本/批量任务正在执行，请先停止')
            return
        path = filedialog.askopenfilename(
            title='选择脚本文件',
            filetypes=[('文本文件', '*.txt'), ('所有文件', '*.*')])
        if not path:
            return

        name = os.path.basename(path)
        try:
            ops, corrected = parse_script(self._read_script_file(path))
        except ScriptError as exc:
            self.log('脚本解析失败（%s）: %s' % (name, exc), 'err')
            messagebox.showerror('脚本解析失败', '%s\n\n%s' % (name, exc), parent=self.root)
            return
        except OSError as exc:
            self.log('读取脚本失败: %s' % exc, 'err')
            messagebox.showerror('读取脚本失败', str(exc), parent=self.root)
            return

        self.script_ops = ops
        self.script_path_var.set(path)
        writes = sum(1 for op in ops if op['op'] == SCRIPT_WRITE)
        self.log('已加载脚本 %s: %d 条操作（写 %d / 读 %d）%s'
                 % (name, len(ops), writes, len(ops) - writes,
                    '，%d 条地址已向下对齐' % corrected if corrected else ''), 'info')
        self._show_script_preview(ops, name, corrected)

    def _show_script_preview(self, ops, name, corrected):
        writes = sum(1 for op in ops if op['op'] == SCRIPT_WRITE)

        win = tk.Toplevel(self.root)
        win.title('脚本预览 - %s' % name)
        win.transient(self.root)
        win.geometry('560x400')
        win.minsize(520, 320)

        body = ttk.Frame(win, padding=8)
        body.pack(fill='both', expand=True)
        body.columnconfigure(0, weight=1)
        body.rowconfigure(1, weight=1)

        ttk.Label(body, text='%s    共 %d 条（写 %d / 读 %d）'
                  % (name, len(ops), writes, len(ops) - writes)).grid(
            row=0, column=0, columnspan=2, sticky='w')

        tree = ttk.Treeview(body, columns=('no', 'op', 'addr', 'data'),
                            show='headings', height=12)
        for key, text, width, anchor in (('no', '#', 48, 'center'),
                                         ('op', '操作', 60, 'center'),
                                         ('addr', '地址', 140, 'w'),
                                         ('data', '数据', 140, 'w')):
            tree.heading(key, text=text)
            tree.column(key, width=width, anchor=anchor, stretch=False)
        tree.grid(row=1, column=0, sticky='nsew', pady=(6, 0))
        scroll = ttk.Scrollbar(body, orient='vertical', command=tree.yview)
        scroll.grid(row=1, column=1, sticky='ns', pady=(6, 0))
        tree.configure(yscrollcommand=scroll.set)

        # 严格按文件顺序，从上到下
        for index, op in enumerate(ops, 1):
            data = '—' if op['value'] is None else '0x%08X' % op['value']
            addr = '0x%08X%s' % (op['addr'], ' *' if op['aligned'] else '')
            tree.insert('', 'end',
                        values=(index, '写' if op['op'] == SCRIPT_WRITE else '读', addr, data))

        if corrected:
            ttk.Label(body, text='* 地址未 4 字节对齐，已向下对齐后执行',
                      foreground='#666666').grid(row=2, column=0, columnspan=2,
                                                 sticky='w', pady=(6, 0))

        btns = ttk.Frame(body)
        btns.grid(row=3, column=0, columnspan=2, sticky='e', pady=(8, 0))
        ttk.Button(btns, text='开始执行', width=10,
                   command=lambda: (win.destroy(),
                                    self._start_script(ops, name))).pack(side='left')
        ttk.Button(btns, text='取消', width=8,
                   command=win.destroy).pack(side='left', padx=(8, 0))
        win.bind('<Escape>', lambda event: win.destroy())

    ##################################################################
    # 执行脚本：后台线程逐行收发，主线程刷新进度与日志
    ##################################################################
    @staticmethod
    def _align_note(op):
        """地址被向下对齐时，在该行日志后附加说明"""
        if not op['aligned']:
            return ''
        return '（地址 0x%08X 已向下对齐）' % op['raw_addr']

    def _start_script(self, ops, name):
        if self.script_active or self.batch_active or not self._require_bus():
            return

        self.script_cancel.clear()
        self.script_active   = True
        self.script_queue    = queue.Queue()
        self.script_ops      = ops
        self._script_started = time.time()

        total  = len(ops)
        writes = sum(1 for op in ops if op['op'] == SCRIPT_WRITE)
        self.script_prog.configure(maximum=total, value=0)
        self.script_count_var.set('0 / %d' % total)
        self.script_stop_btn.configure(state='normal')
        self._set_busy(True)
        self._set_status('● 已连接 · 脚本执行中', self.COLOR_BUSY)

        started = self._script_started
        self._log_plain('[脚本] 开始 %s.%03d  共 %d 条（写 %d / 读 %d）  文件: %s'
                        % (time.strftime('%H:%M:%S', time.localtime(started)),
                           int(started * 1000) % 1000, total,
                           writes, total - writes, name), 'tx')

        def emit(kind, payload):
            self.script_queue.put((kind, payload))

        threading.Thread(target=self._script_worker,
                         args=(ops, emit, self.script_cancel), daemon=True).start()
        self.root.after(40, self._script_poll)

    def _script_worker(self, ops, emit, cancel_evt):
        """后台线程：顺序执行，每行一次 4 字节事务；行边界检查停止请求"""
        total   = len(ops)
        done    = 0
        writes  = 0
        reads   = 0
        stopped = False
        error   = None

        for index, op in enumerate(ops, 1):
            if cancel_evt.is_set():
                stopped = True
                break
            addr = op['addr']
            try:
                if op['op'] == SCRIPT_WRITE:
                    self.bus.write_word(addr, op['value'], 4)
                    writes += 1
                    emit('log', ('[%d/%d] 写 0x%08X <= 0x%08X%s'
                                 % (index, total, addr, op['value'],
                                    self._align_note(op)), 'tx'))
                else:
                    value = self.bus.read_word(addr, 4)
                    reads += 1
                    emit('log', ('[%d/%d] 读 0x%08X => 0x%08X%s'
                                 % (index, total, addr, value,
                                    self._align_note(op)), 'rx'))
                    emit('read', (addr, value))
            except Exception as exc:
                error = (op['line'], str(exc))
                break
            done += 1
            emit('progress', (done, total))

        emit('done', {'done': done, 'total': total, 'writes': writes, 'reads': reads,
                      'stopped': stopped, 'error': error})

    def _script_poll(self):
        """主线程消息泵：刷新进度、输出日志、收尾"""
        if not self.script_active:
            return
        try:
            kind, payload = self.script_queue.get_nowait()
        except queue.Empty:
            self.root.after(40, self._script_poll)
            return

        if kind == 'log':
            self._log_plain(*payload)
            self.root.after(40, self._script_poll)
        elif kind == 'read':
            addr, value = payload
            self.read_var.set('0x%08X' % value)
            self.root.after(40, self._script_poll)
        elif kind == 'progress':
            done, total = payload
            self.script_prog.configure(value=done)
            self.script_count_var.set('%d / %d' % (done, total))
            self.root.after(40, self._script_poll)
        elif kind == 'done':
            self._script_finished(payload)

    def _script_finished(self, payload):
        self.script_active = False
        self.script_cancel.clear()
        self.script_stop_btn.configure(state='disabled')
        self._set_busy(False)

        done, total = payload['done'], payload['total']
        self.script_prog.configure(value=done)
        self.script_count_var.set('%d / %d' % (done, total))
        elapsed  = max(0.0, time.time() - self._script_started)
        finished = time.strftime('%H:%M:%S') + '.%03d' % (int(time.time() * 1000) % 1000)

        if payload['error'] is not None:
            line_no, message = payload['error']
            self._log_plain('[脚本] 第 %d 行失败: %s  完成 %d/%d  用时 %.3f s'
                            % (line_no, message, done, total, elapsed), 'err')
            self._set_status('● 已连接 · 脚本第 %d 行失败' % line_no, self.COLOR_ERR)
            messagebox.showerror('脚本执行失败',
                                 '第 %d 行: %s\n\n已完成 %d/%d'
                                 % (line_no, message, done, total), parent=self.root)
            return

        if payload['stopped']:
            self._log_plain('[脚本] 已停止（用户中断） 完成 %d/%d  用时 %.3f s'
                            % (done, total, elapsed), 'err')
            self._set_status('● 已连接 · 脚本已停止', self.COLOR_BUSY)
            return

        self._log_plain('[脚本] 结束 %s  用时 %.3f s  完成 %d/%d（写 %d / 读 %d）'
                        % (finished, elapsed, done, total,
                           payload['writes'], payload['reads']), 'ok')
        self._set_status('● 已连接 · 脚本完成 (%d 条)' % done, self.COLOR_OK)

    def on_stop_script(self):
        if self.batch_active:
            self.batch_cancel.set()
            self.script_stop_btn.configure(state='disabled')
            self._log_plain('[批量] 已请求停止，当前块结束后退出', 'info')
            return
        if not self.script_active:
            return
        self.script_cancel.set()
        self.script_stop_btn.configure(state='disabled')
        self._log_plain('[脚本] 已请求停止，当前操作结束后退出', 'info')


    ##################################################################
    # 批量：连续 N 点 4 字节写入 / 读取（按块收发，共用底部进度条）
    ##################################################################
    def _batch_busy(self):
        if self.script_active or self.batch_active:
            self._warn('脚本/批量任务正在执行，请先停止')
            return True
        return False

    def _batch_addr_from_ui(self):
        """起始地址（16 进制，可带 0x），未 4 字节对齐时向下对齐"""
        text = self.batch_addr_var.get().strip()
        if not text:
            raise ValueError('起始地址不能为空')
        value = self._parse_int(text, 'hex')
        if value < 0 or value > WAVE_MASK:
            raise ValueError('起始地址超出 32bit 范围（0 ~ 0xFFFFFFFF）')
        aligned = value & ~3
        if aligned != value:
            self.batch_addr_var.set('0x%08X' % aligned)
            self.log('起始地址未 4 字节对齐: 0x%08X -> 0x%08X' % (value, aligned), 'info')
        return aligned

    def _batch_count_from_ui(self, required=True):
        """点数：留空时返回 None（写入时表示用文件里的全部数据）"""
        text = self.batch_count_var.get().strip()
        if not text:
            if required:
                raise ValueError('点数不能为空')
            return None
        try:
            count = int(text, 10)
        except ValueError:
            raise ValueError('点数必须是十进制整数')
        if count <= 0:
            raise ValueError('点数必须大于 0')
        if count > WAVE_MAX_COUNT:
            raise ValueError('点数不能超过 %d' % WAVE_MAX_COUNT)
        return count

    def _batch_block_from_ui(self):
        """每块字数（1..63），决定单帧字节数"""
        text = self.batch_block_var.get().strip()
        try:
            words = int(text, 10) if text else BATCH_BLOCK_WORDS
        except ValueError:
            raise ValueError('每块字数必须是十进制整数')
        if words < 1 or words > BATCH_BLOCK_MAX:
            raise ValueError('每块字数必须在 1 ~ %d 之间（每帧最多 %d 字节）'
                             % (BATCH_BLOCK_MAX, BATCH_BLOCK_MAX * 4))
        return words

    ##################################################################
    # 数据文件 / 输出文件选择
    ##################################################################
    def on_pick_wave_file(self):
        if self._batch_busy():
            return
        initial = os.path.dirname(self.wave_path_var.get().strip()) or os.getcwd()
        path = filedialog.askopenfilename(
            title='选择波表数据文件', initialdir=initial,
            filetypes=[('文本文件', '*.txt'), ('所有文件', '*.*')])
        if not path:
            return
        try:
            values = parse_wave_file(path)
        except WaveError as exc:
            self.log('数据文件解析失败（%s）: %s' % (os.path.basename(path), exc), 'err')
            messagebox.showerror('数据文件解析失败', '%s\n\n%s' % (path, exc),
                                 parent=self.root)
            return
        except OSError as exc:
            self.log('读取数据文件失败: %s' % exc, 'err')
            messagebox.showerror('读取数据文件失败', str(exc), parent=self.root)
            return
        if not values:
            self._warn('数据文件里没有有效数据: %s' % os.path.basename(path))
            return
        self.wave_path_var.set(path)
        self.log('已加载数据文件 %s: %d 个 32bit 数据（点数留空 = 用全部）'
                 % (os.path.basename(path), len(values)), 'info')

    def on_pick_out_file(self):
        if self._batch_busy():
            return
        initial = os.path.dirname(self.wave_out_var.get().strip()
                                  or self.wave_path_var.get().strip()) or os.getcwd()
        path = filedialog.asksaveasfilename(
            title='选择读取输出文件', defaultextension='.txt',
            initialdir=initial, initialfile=self._default_out_name(),
            filetypes=[('文本文件', '*.txt'), ('所有文件', '*.*')])
        if path:
            self.wave_out_var.set(path)

    @staticmethod
    def _default_out_name():
        return 'read_%s.txt' % time.strftime('%Y%m%d_%H%M%S')


    ##################################################################
    # 连续写入：数据文件前 N 点 → 起始地址开始，每点 4 字节
    ##################################################################
    def on_write_wave(self):
        if self._batch_busy() or not self._require_bus():
            return
        path = self.wave_path_var.get().strip()
        if not path:
            self._warn('请先选择数据文件（可用「生成波表…」生成）')
            return
        try:
            addr  = self._batch_addr_from_ui()
            count = self._batch_count_from_ui(required=False)
            block = self._batch_block_from_ui()
        except ValueError as exc:
            self._warn(str(exc))
            return
        try:
            values = parse_wave_file(path)
        except (WaveError, OSError) as exc:
            self._warn('数据文件读取失败: %s（%s）' % (exc, os.path.basename(path)))
            return
        if not values:
            self._warn('数据文件里没有有效数据: %s' % os.path.basename(path))
            return
        if count is None:
            count = len(values)
        if len(values) < count:
            messagebox.showwarning(
                '数据长度不足',
                '数据文件只有 %d 个数据，点数为 %d 个。\n\n文件: %s\n\n'
                '请减小点数或补充数据后再写入。' % (len(values), count, path),
                parent=self.root)
            self.log('数据长度不足: 文件 %d 个 / 需要 %d 个，未写入'
                     % (len(values), count), 'err')
            self._set_status('● 已连接 · 数据长度不足，未写入', self.COLOR_ERR)
            return
        if addr + count * 4 - 1 > WAVE_MASK:
            self._warn('地址范围越界: 0x%08X 起写 %d 点会超过 0xFFFFFFFF'
                       % (addr, count))
            return
        extra = len(values) - count
        if not messagebox.askyesno(
                '确认连续写入',
                '起始地址: 0x%08X\n点数: %d（每点 4 字节，共 %d 字节）\n'
                '地址范围: 0x%08X ~ 0x%08X\n数据文件: %s\n'
                '文件共 %d 个数据，取前 %d 个%s\n\n确定开始写入吗？'
                % (addr, count, count * 4, addr, addr + count * 4 - 1,
                   os.path.basename(path), len(values), count,
                   '，忽略后 %d 个' % extra if extra else ''),
                parent=self.root):
            self.log('已取消连续写入', 'info')
            return
        self._start_batch('write', {'addr': addr, 'count': count,
                                    'block': block, 'values': values, 'path': path})

    ##################################################################
    # 连续读取：起始地址开始读 N 点，结果写入输出文件
    ##################################################################
    def on_read_wave(self):
        if self._batch_busy() or not self._require_bus():
            return
        try:
            addr  = self._batch_addr_from_ui()
            count = self._batch_count_from_ui(required=True)
            block = self._batch_block_from_ui()
        except ValueError as exc:
            self._warn(str(exc))
            return
        if addr + count * 4 - 1 > WAVE_MASK:
            self._warn('地址范围越界: 0x%08X 起读 %d 点会超过 0xFFFFFFFF'
                       % (addr, count))
            return

        path = self.wave_out_var.get().strip()
        if not path:
            initial = os.path.dirname(self.wave_path_var.get().strip()) or os.getcwd()
            path = filedialog.asksaveasfilename(
                title='选择读取输出文件', defaultextension='.txt',
                initialdir=initial, initialfile=self._default_out_name(),
                filetypes=[('文本文件', '*.txt'), ('所有文件', '*.*')])
            if not path:
                return
            self.wave_out_var.set(path)

        if not messagebox.askyesno(
                '确认连续读取',
                '起始地址: 0x%08X\n点数: %d（每点 4 字节，共 %d 字节）\n'
                '地址范围: 0x%08X ~ 0x%08X\n输出文件: %s%s\n\n确定开始读取吗？'
                % (addr, count, count * 4, addr, addr + count * 4 - 1, path,
                   '（覆盖已有文件）' if os.path.exists(path) else ''),
                parent=self.root):
            self.log('已取消连续读取', 'info')
            return
        self._start_batch('read', {'addr': addr, 'count': count,
                                   'block': block, 'path': path})


    ##################################################################
    # 批量执行：后台线程按块收发，主线程刷新进度与日志
    ##################################################################
    def _start_batch(self, mode, params):
        self.batch_cancel.clear()
        self.batch_active   = True
        self.batch_mode     = mode
        self.batch_queue    = queue.Queue()
        self._batch_started = time.time()

        total = params['count']
        name  = '写入' if mode == 'write' else '读取'
        self.script_prog.configure(maximum=total, value=0)
        self.script_count_var.set('0 / %d' % total)
        self.script_stop_btn.configure(state='normal')
        self._set_busy(True)
        self._set_status('● 已连接 · 连续%s中' % name, self.COLOR_BUSY)

        started = self._batch_started
        head = ('[批量] 开始 %s.%03d  连续%s %d 点（%d 字节）  地址 0x%08X  每块 %d 点  '
                % (time.strftime('%H:%M:%S', time.localtime(started)),
                   int(started * 1000) % 1000, name, total, total * 4,
                   params['addr'], params['block']))
        head += ('数据: %s' % os.path.basename(params['path'])) if mode == 'write' \
            else ('输出: %s' % params['path'])
        self._log_plain(head, 'tx' if mode == 'write' else 'rx')

        def emit(kind, payload):
            self.batch_queue.put((kind, payload))

        threading.Thread(target=self._batch_worker,
                         args=(mode, params, emit, self.batch_cancel),
                         daemon=True).start()
        self.root.after(40, self._batch_poll)

    def _batch_worker(self, mode, params, emit, cancel_evt):
        """
        后台线程：按块收发；读取时逐块写入输出文件（停止/失败保留已读部分）。
        日志按 0.5 s 合并发送，避免打断主线程。
        """
        total   = params['count']
        addr    = params['addr']
        block   = params['block']
        values  = params.get('values') or []
        done    = 0
        base    = addr
        stopped = False
        error   = None
        handle  = None
        pending = []
        last    = time.time()

        def flush_logs(force=False):
            nonlocal last
            if pending and (force or time.time() - last >= 0.5):
                for item in pending:
                    emit('log', item)
                del pending[:]
                last = time.time()

        try:
            if mode == 'read':
                handle = open(params['path'], 'w', encoding='utf-8', newline='\n')
            while done < total:
                if cancel_evt.is_set():
                    stopped = True
                    break
                words     = min(block, total - done)
                base      = addr + done * 4
                last_addr = base + words * 4 - 1
                if mode == 'write':
                    payload = b''.join(int(values[done + i]).to_bytes(4, 'little')
                                       for i in range(words))
                    self.bus.write_chunk(base, payload)
                    lines = [wave_line(values[done + i]) for i in range(words)]
                    tag   = 'tx'
                    note  = '[%d/%d] 写 0x%08X..0x%08X  %s' % (
                        done + words, total, base, last_addr, ' '.join(lines))
                else:
                    raw   = self.bus.read_chunk(base, words * 4)
                    lines = [wave_line(int.from_bytes(raw[i * 4:i * 4 + 4], 'little'))
                             for i in range(words)]
                    handle.write('\n'.join(lines) + '\n')
                    handle.flush()
                    tag  = 'rx'
                    note = '[%d/%d] 读 0x%08X..0x%08X  %s' % (
                        done + words, total, base, last_addr, ' '.join(lines))
                done += words
                pending.append((note, tag))
                flush_logs()
                emit('progress', (done, total))
        except Exception as exc:
            error = (base, str(exc))
        finally:
            flush_logs(True)
            if handle is not None:
                try:
                    handle.close()
                except Exception:
                    pass
        emit('done', {'mode': mode, 'done': done, 'total': total,
                      'stopped': stopped, 'error': error, 'path': params['path']})


    ##################################################################
    # 主线程消息泵：一次最多处理 200 条，避免循环期间界面无响应
    ##################################################################
    def _batch_poll(self):
        if not self.batch_active:
            return
        for _ in range(200):
            try:
                kind, payload = self.batch_queue.get_nowait()
            except queue.Empty:
                break
            if kind == 'log':
                self._log_plain(*payload)
            elif kind == 'progress':
                done, total = payload
                self.script_prog.configure(value=done)
                self.script_count_var.set('%d / %d' % (done, total))
            elif kind == 'done':
                self._batch_finished(payload)
                return
        self.root.after(40, self._batch_poll)

    def _batch_finished(self, payload):
        self.batch_active = False
        self.batch_cancel.clear()
        self.script_stop_btn.configure(state='disabled')
        self._set_busy(False)

        mode  = payload['mode']
        done  = payload['done']
        total = payload['total']
        name  = '写入' if mode == 'write' else '读取'
        self.script_prog.configure(value=done)
        self.script_count_var.set('%d / %d' % (done, total))
        elapsed  = max(0.0, time.time() - self._batch_started)
        rate     = (done * 4 / 1024.0) / elapsed if elapsed > 0 else 0.0
        finished = time.strftime('%H:%M:%S') + '.%03d' % (int(time.time() * 1000) % 1000)
        keep     = ('，输出文件保留 %d 点' % done) if mode == 'read' else ''

        if payload['error'] is not None:
            addr, message = payload['error']
            self._log_plain('[批量] 0x%08X 处失败: %s  完成 %d/%d  用时 %.3f s%s'
                            % (addr, message, done, total, elapsed, keep), 'err')
            self._set_status('● 已连接 · 连续%s失败' % name, self.COLOR_ERR)
            detail = '地址 0x%08X: %s\n\n已完成 %d/%d%s' % (
                addr, message, done, total,
                '\n输出文件: %s' % payload['path'] if mode == 'read' else '')
            messagebox.showerror('连续%s失败' % name, detail, parent=self.root)
            return

        if payload['stopped']:
            self._log_plain('[批量] 已停止（用户中断） 完成 %d/%d  用时 %.3f s%s'
                            % (done, total, elapsed, keep), 'err')
            self._set_status('● 已连接 · 连续%s已停止' % name, self.COLOR_BUSY)
            return

        tail = ('  → %s' % payload['path']) if mode == 'read' else ''
        self._log_plain('[批量] 结束 %s  用时 %.3f s  连续%s %d/%d 点（%d 字节）  '
                        '平均 %.1f KB/s%s'
                        % (finished, elapsed, name, done, total, done * 4, rate, tail), 'ok')
        self._set_status('● 已连接 · 连续%s完成 (%d 点)' % (name, done), self.COLOR_OK)
        if mode == 'read':
            self.log('读取结果已保存: %s' % payload['path'], 'info')


    ##################################################################
    # 波表生成对话框：f(n) → N 点 → 8 位 16 进制文本
    ##################################################################
    def on_open_wave_gen(self):
        win = self.wave_win
        if win is not None and win.winfo_exists():
            win.deiconify()
            win.lift()
            win.focus_force()
            return
        self._show_wave_gen()

    def _show_wave_gen(self):
        win = tk.Toplevel(self.root)
        self.wave_win = win
        win.title('波表生成')
        win.transient(self.root)
        win.geometry('700x700')
        win.minsize(640, 620)

        body = ttk.Frame(win, padding=8)
        body.pack(fill='both', expand=True)
        body.columnconfigure(0, weight=1)
        body.rowconfigure(1, weight=1)

        count_var  = tk.StringVar(value='1024')
        amp_var    = tk.StringVar(value=str(WAVE_MAX))
        offset_var = tk.StringVar(value='0')
        expr_var   = tk.StringVar(value=WAVE_TEMPLATES[0][1])
        stat_var   = tk.StringVar(value='—')
        prog_text  = tk.StringVar(value='')

        # ---- 参数 ----
        param = ttk.LabelFrame(body, text=' 参数 ', padding=8)
        param.grid(row=0, column=0, sticky='ew')
        param.columnconfigure(1, weight=1)

        top = ttk.Frame(param)
        top.grid(row=0, column=0, columnspan=3, sticky='w')
        ttk.Label(top, text='点数 N').pack(side='left')
        count_entry = ttk.Entry(top, textvariable=count_var, width=10,
                                font=('Consolas', 10))
        count_entry.pack(side='left', padx=(6, 12))
        ttk.Label(top, text='幅值 A').pack(side='left')
        amp_entry = ttk.Entry(top, textvariable=amp_var, width=11,
                              font=('Consolas', 10))
        amp_entry.pack(side='left', padx=(6, 12))
        ttk.Label(top, text='偏移 O').pack(side='left')
        offset_entry = ttk.Entry(top, textvariable=offset_var, width=11,
                                 font=('Consolas', 10))
        offset_entry.pack(side='left', padx=(6, 12))
        template = ttk.Combobox(top, state='readonly', width=10,
                                values=[name for name, _ in WAVE_TEMPLATES])
        template.current(0)
        template.pack(side='left')

        ttk.Label(param, text='函数 f(n)').grid(row=1, column=0, sticky='w', pady=(8, 0))
        expr_entry = ttk.Entry(param, textvariable=expr_var, font=('Consolas', 10))
        expr_entry.grid(row=1, column=1, columnspan=2, sticky='ew', padx=(6, 0),
                        pady=(8, 0))
        ttk.Label(param,
                  text='结果 = f(n) + O，四舍五入后截断到 32bit 有符号范围；可用 n、N、pi、e、A，'
                       '以及 sin cos tan asin acos atan atan2 sqrt fabs exp log log10 '
                       'floor ceil round sign abs min max，条件写成 a if c else b',
                  foreground='#666666', wraplength=600, justify='left').grid(
            row=2, column=0, columnspan=3, sticky='w', pady=(6, 0))

        # ---- 数据预览：点数很多，逐点表格没有意义，只画波形轮廓 ----
        prev = ttk.LabelFrame(body, text=' 波形预览 ', padding=6)
        prev.grid(row=1, column=0, sticky='nsew', pady=(8, 0))
        prev.columnconfigure(0, weight=1)
        prev.rowconfigure(0, weight=1)
        plot = tk.Canvas(prev, height=300, background='#FFFFFF',
                         highlightthickness=1, highlightbackground='#C8C8C8')
        plot.grid(row=0, column=0, sticky='nsew')

        stat = ttk.Label(body, textvariable=stat_var, wraplength=660, justify='left')
        stat.grid(row=2, column=0, sticky='w', pady=(6, 0))

        prog_box = ttk.Frame(body)
        prog_box.grid(row=3, column=0, sticky='ew', pady=(6, 0))
        prog_bar = ttk.Progressbar(prog_box, orient='horizontal', mode='determinate',
                                   maximum=1)
        prog_bar.pack(side='left', fill='x', expand=True)
        ttk.Label(prog_box, textvariable=prog_text, width=16).pack(
            side='left', padx=(8, 0))
        prog_box.grid_remove()

        btns = ttk.Frame(body)
        btns.grid(row=4, column=0, sticky='e', pady=(8, 0))
        preview_btn = ttk.Button(btns, text='刷新预览', width=10,
                                 command=lambda: preview())
        preview_btn.pack(side='left')
        gen_btn = ttk.Button(btns, text='生成到文件…', width=13,
                             command=lambda: generate())
        gen_btn.pack(side='left', padx=(8, 0))
        close_btn = ttk.Button(btns, text='关闭', width=8, command=lambda: close())
        close_btn.pack(side='left', padx=(8, 0))

        gen = {'active': False, 'path': '', 'expr': '', 'amp': 0, 'offset': 0,
               'cancel': None, 'queue': queue.Queue()}

        def set_stat(text, color='#333333'):
            stat_var.set(text)
            stat.configure(foreground=color)

        def parse_fields():
            text = count_var.get().strip()
            try:
                count = int(text, 10) if text else 0
            except ValueError:
                raise WaveError('点数必须是十进制整数')
            if count < 1 or count > WAVE_MAX_COUNT:
                raise WaveError('点数必须在 1 ~ %d 之间' % WAVE_MAX_COUNT)
            amp = parse_wave_int(amp_var.get(), '幅值 A')
            if amp < 0 or amp > WAVE_MAX:
                raise WaveError('幅值 A 必须在 0 ~ %d 之间' % WAVE_MAX)
            offset = parse_wave_int(offset_var.get(), '偏移 O', 0)
            if offset < WAVE_MIN or offset > WAVE_MAX:
                raise WaveError('偏移 O 必须在 %d ~ %d 之间' % (WAVE_MIN, WAVE_MAX))
            code = compile_wave_expr(expr_var.get())
            return count, amp, offset, code

        shown = {'code': None, 'count': 0, 'amp': 0, 'offset': 0,
                 'low': 0, 'high': 0}

        def draw_plot():
            info = shown
            if info['code'] is None:
                draw_wave_canvas(plot, 0, 0, 0, None,
                                 '点「刷新预览」在此画出波形轮廓')
                return
            draw_wave_canvas(plot, info['low'], info['high'], info['count'],
                             lambda cols: wave_profile(info['code'], info['count'],
                                                       info['amp'], info['offset'],
                                                       cols))

        plot.bind('<Configure>', lambda event: draw_plot())

        def preview():
            if gen['active']:
                return None
            try:
                count, amp, offset, code = parse_fields()
            except WaveError as exc:
                shown['code'] = None
                draw_plot()
                set_stat(str(exc), self.COLOR_ERR)
                return None

            sample = min(count, WAVE_STAT_SAMPLE)
            try:
                low = high = None
                total_value = 0
                truncated = 0
                for index in range(sample):
                    value, cut = eval_wave_point(code, index, count, amp, offset)
                    if cut:
                        truncated += 1
                    low = value if low is None or value < low else low
                    high = value if high is None or value > high else high
                    total_value += value
            except WaveError as exc:
                shown['code'] = None
                draw_plot()
                set_stat(str(exc), self.COLOR_ERR)
                return None

            shown.update({'code': code, 'count': count, 'amp': amp,
                          'offset': offset, 'low': low, 'high': high})
            draw_plot()
            mean = int(round(total_value / float(sample)))
            note = '' if sample == count else '（统计前 %d 点）' % sample
            set_stat('共 %d 点，轮廓覆盖全部点%s  最小 %s (%d)   '
                     '最大 %s (%d)   均值 %d   截断 %d 点'
                     % (count, note, wave_line(low), low,
                        wave_line(high), high, mean, truncated),
                     self.COLOR_BUSY if truncated else '#333333')
            return count, amp, offset, code

        def set_generating(active):
            gen['active'] = active
            state = 'disabled' if active else 'normal'
            preview_btn.configure(state=state)
            gen_btn.configure(state=state)
            close_btn.configure(text='取消' if active else '关闭', state='normal')
            if active:
                prog_box.grid()
            else:
                prog_box.grid_remove()
                self._wave_cancel = None

        def remove_quietly(path):
            try:
                if path and os.path.exists(path):
                    os.remove(path)
            except OSError:
                pass

        def start_generation(path, fields):
            count, amp, offset, _code = fields
            expr = expr_var.get().strip()
            cancel_evt = threading.Event()
            win_queue = queue.Queue()
            gen.update({'path': path, 'expr': expr, 'amp': amp, 'offset': offset,
                        'cancel': cancel_evt, 'queue': win_queue})
            self._wave_cancel = cancel_evt
            prog_bar.configure(maximum=count, value=0)
            prog_text.set('0 / %d 点' % count)
            set_stat('正在生成 %d 点 → %s' % (count, path))
            set_generating(True)

            def runner():
                try:
                    written, truncated = write_wave_file(
                        path, expr, count, amp, offset,
                        on_progress=lambda done, total: win_queue.put(
                            ('progress', (done, total))),
                        cancel=cancel_evt)
                except WaveCancelled:
                    win_queue.put(('cancel', None))
                except Exception as exc:
                    win_queue.put(('error', str(exc)))
                else:
                    win_queue.put(('done', (written, truncated)))

            threading.Thread(target=runner, daemon=True).start()
            win.after(60, poll)

        def poll():
            if not win.winfo_exists():
                return
            try:
                while True:
                    kind, payload = gen['queue'].get_nowait()
                    if kind == 'progress':
                        done, total = payload
                        prog_bar.configure(maximum=total, value=done)
                        prog_text.set('%d / %d 点' % (done, total))
                    elif kind == 'done':
                        written, truncated = payload
                        set_generating(False)
                        prog_bar.configure(value=written)
                        prog_text.set('完成')
                        set_stat('已保存 %d 点 → %s%s'
                                 % (written, gen['path'],
                                    '，截断 %d 点' % truncated if truncated else ''),
                                 self.COLOR_OK)
                        self.wave_path_var.set(gen['path'])
                        self.log('[波表] %d 点 A=%d O=%d f(n)=%s → %s%s'
                                 % (written, gen['amp'], gen['offset'], gen['expr'],
                                    gen['path'],
                                    '（截断 %d 点）' % truncated if truncated else ''),
                                 'info')
                        return
                    elif kind == 'cancel':
                        remove_quietly(gen['path'])
                        set_generating(False)
                        prog_text.set('已取消')
                        set_stat('已取消，未保留文件', self.COLOR_BUSY)
                        self.log('[波表] 已取消生成，未保留文件', 'info')
                        return
                    elif kind == 'error':
                        remove_quietly(gen['path'])
                        set_generating(False)
                        prog_text.set('失败')
                        set_stat('生成失败: %s' % payload, self.COLOR_ERR)
                        self.log('[波表] 生成失败: %s' % payload, 'err')
                        messagebox.showerror('波表生成失败', payload, parent=win)
                        return
            except queue.Empty:
                pass
            win.after(60, poll)

        def generate():
            if gen['active']:
                return
            fields = preview()
            if fields is None:
                return
            initial = os.path.dirname(self.wave_path_var.get().strip()) or os.getcwd()
            path = filedialog.asksaveasfilename(
                parent=win, title='保存波表文件', defaultextension='.txt',
                initialdir=initial, initialfile='wave_%d.txt' % fields[0],
                filetypes=[('文本文件', '*.txt'), ('所有文件', '*.*')])
            if path:
                start_generation(path, fields)

        def close():
            if gen['active'] and gen['cancel'] is not None:
                gen['cancel'].set()
                close_btn.configure(state='disabled')
                set_stat('正在取消…', self.COLOR_BUSY)
                return
            win.destroy()

        def on_template(event=None):
            chosen = dict(WAVE_TEMPLATES).get(template.get())
            if chosen:
                expr_var.set(chosen)
                preview()

        template.bind('<<ComboboxSelected>>', on_template)
        for widget in (count_entry, amp_entry, offset_entry, expr_entry):
            widget.bind('<Return>', lambda event: preview())
        win.bind('<Escape>', lambda event: close())
        win.protocol('WM_DELETE_WINDOW', close)
        try:
            preview()
        except Exception as exc:
            set_stat('预览失败: %s' % exc, self.COLOR_ERR)

    ##################################################################
    # 文件波形查看：把「数据文件」/「输出文件」里的真实数据画成波形
    # （写入前确认即将写进去的波形，读取后确认读回来的波形）
    ##################################################################
    def on_show_wave_plot(self):
        """打开波形查看窗：显示当前数据文件（存在时），否则输出文件"""
        win = self.wave_plot_win
        if win is not None:
            try:
                alive = win.winfo_exists()
            except Exception:
                alive = False
            if alive:
                win.deiconify()
                win.lift()
                win.focus_force()
                self._plot_wave_file(self._default_plot_file())
                return
            self.wave_plot_win    = None
            self.wave_plot_canvas = None
        self._show_wave_plot(self._default_plot_file())

    def _default_plot_file(self):
        """默认看哪个文件：数据文件存在就看数据文件，否则输出文件"""
        for path in (self.wave_path_var.get().strip(),
                     self.wave_out_var.get().strip()):
            if path and os.path.exists(path):
                return path
        return ''

    def _show_wave_plot(self, path):
        win = tk.Toplevel(self.root)
        self.wave_plot_win = win
        win.title('波形查看')
        win.transient(self.root)
        win.geometry('760x640')
        win.minsize(620, 420)

        body = ttk.Frame(win, padding=8)
        body.pack(fill='both', expand=True)
        body.columnconfigure(0, weight=1)
        body.rowconfigure(1, weight=1)

        self.wave_plot_path_var = tk.StringVar(value='')
        self.wave_plot_stat_var = tk.StringVar(value='—')
        self.wave_plot_data     = {'values': [], 'low': 0, 'high': 0}

        row = ttk.Frame(body)
        row.grid(row=0, column=0, sticky='ew')
        row.columnconfigure(1, weight=1)
        ttk.Label(row, text='文件').grid(row=0, column=0, sticky='w')
        ttk.Entry(row, textvariable=self.wave_plot_path_var, state='readonly',
                  font=('Consolas', 10)).grid(row=0, column=1, sticky='ew',
                                              padx=(6, 6))
        ttk.Button(row, text='打开…', width=7,
                   command=self.on_open_plot_file).grid(row=0, column=2)
        ttk.Button(row, text='刷新', width=6,
                   command=self.on_reload_plot).grid(row=0, column=3, padx=(6, 0))

        plot = tk.Canvas(body, height=420, background='#FFFFFF',
                         highlightthickness=1, highlightbackground='#C8C8C8')
        plot.grid(row=1, column=0, sticky='nsew', pady=(8, 0))
        self.wave_plot_canvas = plot
        plot.bind('<Configure>', lambda event: self._draw_wave_plot())

        self.wave_plot_stat_label = tk.Label(
            body, textvariable=self.wave_plot_stat_var, anchor='w', justify='left',
            fg=self.COLOR_IDLE, font=('Microsoft YaHei UI', 9))
        self.wave_plot_stat_label.grid(row=2, column=0, sticky='ew', pady=(6, 0))

        box = ttk.Frame(body)
        box.grid(row=3, column=0, sticky='ew', pady=(8, 0))
        ttk.Button(box, text='关闭', width=8,
                   command=self._close_wave_plot).pack(side='right')
        ttk.Label(box, text='自动显示数据文件（存在时）或输出文件；'
                            '「打开…」可看任意文件，「刷新」重读',
                  foreground='#666666').pack(side='left')

        win.bind('<Escape>', lambda event: self._close_wave_plot())
        win.protocol('WM_DELETE_WINDOW', self._close_wave_plot)
        self._plot_wave_file(path)

    def on_open_plot_file(self):
        """选择任意数据文件查看波形"""
        current = self.wave_plot_path_var.get().strip()
        initial = os.path.dirname(current or self.wave_path_var.get().strip()
                                  or self.wave_out_var.get().strip()) or os.getcwd()
        path = filedialog.askopenfilename(
            parent=self.wave_plot_win, title='选择波形数据文件',
            initialdir=initial,
            filetypes=[('文本文件', '*.txt'), ('所有文件', '*.*')])
        if path:
            self._plot_wave_file(path)

    def on_reload_plot(self):
        """重新读取当前文件（写入/读取完成后可刷新）"""
        self._plot_wave_file(self.wave_plot_path_var.get().strip())

    def _close_wave_plot(self):
        """关闭查看窗"""
        win                   = self.wave_plot_win
        self.wave_plot_win    = None
        self.wave_plot_canvas = None
        if win is not None:
            try:
                win.destroy()
            except Exception:
                pass

    def _set_plot_stat(self, text, color=None):
        if self.wave_plot_stat_var is None:
            return
        self.wave_plot_stat_var.set(text)
        label = getattr(self, 'wave_plot_stat_label', None)
        if label is not None:
            try:
                label.configure(fg=color or self.COLOR_IDLE)
            except Exception:
                pass

    def _plot_wave_file(self, path):
        """
        解析文件并画出真实波形（统计按全部点计算）。
        path 为空 / 文件不存在 / 解析失败时只显示提示，不动其他控件。
        """
        plot = self.wave_plot_canvas
        if plot is None:
            return
        try:
            if not plot.winfo_exists():
                return
        except Exception:
            return

        self.wave_plot_path_var.set(path or '')
        self.wave_plot_data = {'values': [], 'low': 0, 'high': 0}

        if not path:
            self._set_plot_stat('尚未选择文件：点「打开…」选一个，或先用「选择…」/「另存…」'
                                '指定数据文件 / 输出文件', self.COLOR_ERR)
            self._draw_wave_plot()
            return
        if not os.path.exists(path):
            self._set_plot_stat('文件不存在: %s' % path, self.COLOR_ERR)
            self._draw_wave_plot()
            return

        self.root.configure(cursor='watch')
        self.root.update_idletasks()
        try:
            patterns = parse_wave_file(path)
        except (WaveError, OSError) as exc:
            self._set_plot_stat('%s 解析失败: %s'
                                % (os.path.basename(path), exc), self.COLOR_ERR)
            self.log('波形查看失败（%s）: %s' % (os.path.basename(path), exc), 'err')
            self._draw_wave_plot()
            return
        finally:
            self.root.configure(cursor='')

        if not patterns:
            self._set_plot_stat('%s 里没有有效数据' % os.path.basename(path),
                                self.COLOR_ERR)
            self._draw_wave_plot()
            return

        values = [wave_signed(item) for item in patterns]
        low  = min(values)
        high = max(values)
        mean = _wave_round(sum(values) / float(len(values)))
        self.wave_plot_data = {'values': values, 'low': low, 'high': high}
        self._draw_wave_plot()
        try:
            size = os.path.getsize(path)
        except OSError:
            size = 0
        self._set_plot_stat(
            '共 %d 点（%d 字节，%s）  最小 %d (0x%s)  最大 %d (0x%s)  均值 %d'
            % (len(values), size, os.path.basename(path),
               low, wave_line(low), high, wave_line(high), mean),
            self.COLOR_OK)
        self.log('已查看波形 %s: %d 点，最小 %d，最大 %d'
                 % (os.path.basename(path), len(values), low, high), 'info')

    def _draw_wave_plot(self):
        """按当前数据重画 Canvas（窗口缩放时也会调用）"""
        plot = self.wave_plot_canvas
        if plot is None:
            return
        try:
            if not plot.winfo_exists():
                return
        except Exception:
            return
        data   = self.wave_plot_data or {}
        values = data.get('values') or []
        if not values:
            draw_wave_canvas(plot, 0, 0, 0, None,
                             '选择数据文件或输出文件后在此显示真实波形'
                             '（纵轴按数据范围自动缩放）')
            return
        draw_wave_canvas(plot, data['low'], data['high'], len(values),
                         lambda cols: wave_profile_values(values, cols))


##################################################################
# Main
##################################################################
def main(argv):
    parser = argparse.ArgumentParser(description='UART -> AXI 32bit 寄存器调试工具')
    parser.add_argument('-d', dest='device',  default='',        help='串口设备，例如 COM3')
    parser.add_argument('-b', dest='baud',    default=1000000,   type=int,   help='波特率（默认 1000000）')
    parser.add_argument('-t', dest='timeout', default=0.5,       type=float, help='串口读超时秒数（默认 0.5）')
    args = parser.parse_args(argv)

    root = tk.Tk()
    try:
        ttk.Style().theme_use('vista')
    except Exception:
        pass
    DebugApp(root, args.device, args.baud, args.timeout)
    root.mainloop()


if __name__ == '__main__':
    main(sys.argv[1:])
