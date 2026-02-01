#!/usr/bin/env python3
"""
CSV Format Validator and Fixer for GitHub Issues
Validates and fixes CSV formatting: ensures commas are present and fields are properly quoted
"""

import csv
import re
import sys
from pathlib import Path


def fix_and_validate_csv(csv_file):
    """
    Fix CSV format issues and validate the result.
    
    Args:
        csv_file: Path to the CSV file to validate and fix
        
    Returns:
        tuple: (is_valid, errors list, fixes_applied list)
    """
    errors = []
    fixes = []
    
    if not Path(csv_file).exists():
        return False, [f"Error: CSV file '{csv_file}' not found"], []
    
    try:
        with open(csv_file, 'r', encoding='utf-8') as f:
            lines = f.read().strip().split('\n')
        
        if not lines:
            return False, ["Error: CSV file is empty"], []
        
        # Parse header to get expected field count
        header_line = lines[0]
        try:
            reader = csv.reader([header_line])
            headers = next(reader)
        except Exception as e:
            return False, [f"Error parsing header: {e}"], []
        
        num_expected_fields = len(headers)
        fixed_lines = []
        
        # Fix header line first - all fields except 'number' should be quoted
        fixed_header = fix_csv_line(header_line, num_expected_fields, headers, is_header=True)
        if fixed_header != header_line:
            fixes.append(f"Line 1 (header): Added missing quotation marks")
        fixed_lines.append(fixed_header)
        
        # Process data lines
        for line_num, line in enumerate(lines[1:], start=2):
            if not line.strip():
                fixed_lines.append(line)
                continue
            
            try:
                reader = csv.reader([line])
                row = next(reader)
            except Exception as e:
                errors.append(f"Line {line_num}: Failed to parse - {e}")
                fixed_lines.append(line)
                continue
            
            # Check if we have the right number of fields
            if len(row) != num_expected_fields:
                errors.append(f"Line {line_num}: Expected {num_expected_fields} fields, got {len(row)}")
                fixed_lines.append(line)
                continue
            
            # Fix the line to add proper quoting
            fixed_line = fix_csv_line(line, num_expected_fields, headers)
            if fixed_line != line:
                fixes.append(f"Line {line_num}: Added missing quotation marks around fields")
            fixed_lines.append(fixed_line)
        
        # Write fixed CSV back to file if any fixes were applied
        if fixes:
            with open(csv_file, 'w', encoding='utf-8', newline='') as f:
                f.write('\n'.join(fixed_lines) + '\n')
        
    except Exception as e:
        errors.append(f"Error reading/writing file: {e}")
        return False, errors, fixes
    
    is_valid = len(errors) == 0
    return is_valid, errors, fixes


def fix_csv_line(line, expected_fields, headers, is_header=False):
    """
    Fix a CSV line by ensuring proper quoting on all fields except 'number'.
    
    Args:
        line: The CSV line to fix
        expected_fields: Expected number of fields
        headers: The header fields
        is_header: Whether this is a header line
        
    Returns:
        str: Fixed CSV line with proper quoting
    """
    # Parse the line with csv reader
    try:
        reader = csv.reader([line])
        row = next(reader)
    except:
        return line
    
    if len(row) != expected_fields:
        return line  # Can't fix if field count is wrong
    
    # Reconstruct the line with proper quoting
    fixed_parts = []
    for i, value in enumerate(row):
        field_name = headers[i] if i < len(headers) else f"field_{i}"
        
        if value and value.startswith('"') and value.endswith('"'):
            fixed_parts.append(value)
        else:
            # Escape any internal quotes
            escaped_value = value.replace('"', '""') if value else ""
            fixed_parts.append(f'"{escaped_value}"')
    
    return ','.join(fixed_parts)


def print_report(csv_file, is_valid, errors, fixes):
    """Print a formatted validation and fix report."""
    print("\n" + "=" * 70)
    print(f"CSV Validation & Fix Report: {csv_file}")
    print("=" * 70)
    
    if fixes:
        print(f"\n✓ {len(fixes)} issue(s) FIXED:")
        for fix in fixes:
            print(f"  • {fix}")
    
    if is_valid:
        print("\n✓ CSV format is VALID")
    else:
        print("\n✗ CSV format has remaining ERRORS:")
        for error in errors:
            print(f"  • {error}")
    
    print("\n" + "=" * 70 + "\n")
    return is_valid


def main():
    """Main entry point."""
    if len(sys.argv) < 2:
        csv_file = "scripts/github_issues/github_issues_to_update.csv"
        print(f"No CSV file specified. Using default: {csv_file}")
    else:
        csv_file = sys.argv[1]
    
    is_valid, errors, fixes = fix_and_validate_csv(csv_file)
    print_report(csv_file, is_valid, errors, fixes)
    
    sys.exit(0 if is_valid else 1)


if __name__ == "__main__":
    main()
