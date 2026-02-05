
import sys
import os


files = [
    r"d:\robot\FOC\FalconFoc-main\2.Firmware\FalconFoc\MODULES\motor\calibration\calib_inductance.c",
    r"d:\robot\FOC\FalconFoc-main\2.Firmware\FalconFoc\MODULES\motor\calibration\calib_inductance.h",
    r"d:\robot\FOC\FalconFoc-main\2.Firmware\FalconFoc\MODULES\motor\calibration\calib_resistance.c",
    r"d:\robot\FOC\FalconFoc-main\2.Firmware\FalconFoc\MODULES\motor\calibration\calib_resistance.h",
    r"d:\robot\FOC\FalconFoc-main\2.Firmware\FalconFoc\MODULES\motor\calibration\calibration_context.c",
    r"d:\robot\FOC\FalconFoc-main\2.Firmware\FalconFoc\MODULES\motor\calibration\calibration_context.h",
    r"d:\robot\FOC\FalconFoc-main\2.Firmware\FalconFoc\MODULES\motor\calibration\current_calib.c",
    r"d:\robot\FOC\FalconFoc-main\2.Firmware\FalconFoc\MODULES\motor\calibration\current_calib.h",
    r"d:\robot\FOC\FalconFoc-main\2.Firmware\FalconFoc\MODULES\motor\calibration\flux_calib.c",
    r"d:\robot\FOC\FalconFoc-main\2.Firmware\FalconFoc\MODULES\motor\calibration\flux_calib.h",
    r"d:\robot\FOC\FalconFoc-main\2.Firmware\FalconFoc\MODULES\motor\calibration\rsls_calib.c",
    r"d:\robot\FOC\FalconFoc-main\2.Firmware\FalconFoc\MODULES\motor\calibration\rsls_calib.h",
]


for fpath in files:
    try:
        with open(fpath, 'rb') as f:
            raw = f.read()
        
        # Try decoding as GB18030 first (common for Chinese)
        try:
            content = raw.decode('gb18030')
            print(f"Decoded {fpath} with GB18030")
        except:
            # Fallback to utf-8 (or replace)
            content = raw.decode('utf-8', errors='replace')
            print(f"Decoded {fpath} with UTF-8 (fallback)")
            
        dest = fpath + ".utf8.txt"
        with open(dest, 'w', encoding='utf-8') as f:
            f.write(content)
        print(f"Wrote to {dest}")
        
    except Exception as e:
        print(f"Error processing {fpath}: {e}")
