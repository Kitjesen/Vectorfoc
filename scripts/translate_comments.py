#!/usr/bin/env python3
"""
Convert Chinese comments to English in VectorFOC source files.
"""

import os
import re

# Common Chinese to English translations for FOC comments
TRANSLATIONS = {
    # General
    "电机": "motor",
    "电流": "current",
    "电压": "voltage",
    "角度": "angle",
    "速度": "speed/velocity",
    "位置": "position",
    "转速": "speed",
    "扭矩": "torque",
    "功率": "power",
    "温度": "temperature",
    "状态": "state",
    "模式": "mode",
    "使能": "enable",
    "禁用": "disable",
    "启动": "start",
    "停止": "stop",
    "初始化": "init",
    "配置": "config",
    "参数": "param",
    "输入": "input",
    "输出": "output",
    "计算": "calc",
    "更新": "update",
    "获取": "get",
    "设置": "set",
    "检查": "check",
    "校准": "calibration",
    "限制": "limit",
    "滤波": "filter",
    "采样": "sample",
    "周期": "period",
    "频率": "frequency",
    "增益": "gain",
    "误差": "error",
    "偏移": "offset",
    "阈值": "threshold",
    "饱和": "saturation",
    "过载": "overload",
    "故障": "fault",
    "保护": "protection",
    "安全": "safety",
    "警告": "warning",
    "正常": "normal",
    "异常": "abnormal",
    
    # FOC specific
    "磁链": "flux",
    "极对数": "pole pairs",
    "相电阻": "phase resistance",
    "相电感": "phase inductance",
    "反电动势": "back-EMF",
    "占空比": "duty cycle",
    "调制": "modulation",
    "过调制": "overmodulation",
    "解耦": "decoupling",
    "前馈": "feedforward",
    "积分": "integral",
    "比例": "proportional",
    "微分": "derivative",
    "抗积分饱和": "anti-windup",
    "编码器": "encoder",
    "霍尔": "hall",
    "无感": "sensorless",
    "观测器": "observer",
    "滑模": "sliding mode",
    "锁相环": "PLL",
    
    # Control
    "电流环": "current loop",
    "速度环": "speed loop",
    "位置环": "position loop",
    "内环": "inner loop",
    "外环": "outer loop",
    "闭环": "closed loop",
    "开环": "open loop",
    "轴": "axis",
    "参考值": "reference",
    "反馈": "feedback",
    "目标": "target",
    "实际": "actual",
    
    # State machine
    "空闲": "idle",
    "运行": "running",
    "就绪": "ready",
    "错误": "error",
    "复位": "reset",
    "等待": "wait",
    "完成": "done",
    "超时": "timeout",
    
    # Hardware
    "相": "phase",
    "三相": "three-phase",
    "逆变器": "inverter",
    "驱动": "driver",
    "采集": "acquisition",
    "中断": "interrupt",
    "定时器": "timer",
    "寄存器": "register",
}

def translate_comment(text):
    """Translate Chinese text to English."""
    result = text
    for cn, en in TRANSLATIONS.items():
        result = result.replace(cn, en)
    return result

def has_chinese(text):
    """Check if text contains Chinese characters."""
    return bool(re.search(r'[\u4e00-\u9fff]', text))

def process_file(filepath):
    """Process a single file, translating Chinese comments."""
    try:
        with open(filepath, 'r', encoding='utf-8') as f:
            content = f.read()
    except UnicodeDecodeError:
        try:
            with open(filepath, 'r', encoding='gbk') as f:
                content = f.read()
        except:
            print(f"  [SKIP] Cannot read: {filepath}")
            return False
    
    if not has_chinese(content):
        return False
    
    lines = content.split('\n')
    modified = False
    new_lines = []
    
    for line in lines:
        if has_chinese(line):
            # Check if it's a comment line
            if '//' in line or '/*' in line or '*' in line.lstrip():
                new_line = translate_comment(line)
                if new_line != line:
                    modified = True
                    line = new_line
        new_lines.append(line)
    
    if modified:
        with open(filepath, 'w', encoding='utf-8') as f:
            f.write('\n'.join(new_lines))
        return True
    return False

def main():
    import argparse
    parser = argparse.ArgumentParser(
        description="Translate Chinese comments to English in VectorFOC source files."
    )
    parser.add_argument(
        "src_dir",
        nargs="?",
        default=os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "Src"),
        help="Path to source directory (default: ../Src relative to this script)",
    )
    args = parser.parse_args()
    src_dir = os.path.abspath(args.src_dir)
    
    modified_count = 0
    total_count = 0
    
    for root, dirs, files in os.walk(src_dir):
        for file in files:
            if file.endswith(('.c', '.h')):
                filepath = os.path.join(root, file)
                total_count += 1
                if process_file(filepath):
                    print(f"  [MOD] {filepath}")
                    modified_count += 1
    
    print(f"\nProcessed {total_count} files, modified {modified_count}")

if __name__ == "__main__":
    main()
