#!/usr/bin/env python3
"""
Remove all Chinese characters from comments in VectorFOC source files.
Keeps the comment structure but removes Chinese text.
"""

import os
import re

def remove_chinese(text):
    """Remove Chinese characters from text."""
    return re.sub(r'[\u4e00-\u9fff]+', '', text)

def process_file(filepath):
    """Process a single file, removing Chinese from comments."""
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
    
    if not re.search(r'[\u4e00-\u9fff]', content):
        return False
    
    # Remove Chinese characters
    new_content = remove_chinese(content)
    
    # Clean up empty comments and extra spaces
    new_content = re.sub(r'///\s*\n', '///\n', new_content)  # Empty /// comments
    new_content = re.sub(r'/\*\*\s*\*/', '/** */', new_content)  # Empty /** */
    new_content = re.sub(r'\s+\n', '\n', new_content)  # Trailing spaces
    
    if new_content != content:
        with open(filepath, 'w', encoding='utf-8') as f:
            f.write(new_content)
        return True
    return False

def main():
    import argparse
    parser = argparse.ArgumentParser(
        description="Remove Chinese characters from comments in VectorFOC source files."
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
