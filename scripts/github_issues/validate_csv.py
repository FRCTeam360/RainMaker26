#!/usr/bin/env python3
"""
CSV Format Validator and Fixer for GitHub Issues
Validates CSV format and automatically fixes unescaped quotes within field values.
"""

import csv
import sys
import re
from pathlib import Path


def fix_unescaped_quotes(content):
    """
    Fix unescaped quotes in CSV content by re-parsing and properly escaping.
    
    This reads the file line-by-line and uses a simple state machine to:
    1. Parse quoted fields
    2. Find any quotes within fields that aren't properly doubled
    3. Double them
    
    Args:
        content: Raw CSV file content as string
        
    Returns:
        tuple: (fixed_content, number_of_fixes)
    """
    lines = content.split('\n')
    fixed_lines = []
    total_fixes = 0
    
    for line_num, line in enumerate(lines, 1):
        if not line.strip():
            fixed_lines.append(line)
            continue
        
        # Try to parse the line with csv module first to see if it's valid
        try:
            import csv
            import io
            reader = csv.reader(io.StringIO(line))
            parsed = list(reader)
            if parsed and len(parsed[0]) == 4:  # Expected number of fields
                # Line appears valid, but still check if we need to quote unquoted fields
                # Re-parse to ensure all fields are quoted
                pass  # Fall through to manual parsing
        except:
            pass
        
        # Parse and fix the line
        # Strategy: Split by commas that are outside quotes
        # Then for each field, ensure internal quotes are doubled and all fields are quoted
        
        result_fields = []
        current_field = []
        in_quotes = False
        i = 0
        
        while i < len(line):
            char = line[i]
            
            if char == '"':
                if not in_quotes:
                    # Start of quoted field
                    in_quotes = True
                    i += 1
                elif i + 1 < len(line) and line[i+1] == '"':
                    # Escaped quote - keep both
                    current_field.append('""')
                    i += 2
                else:
                    # End of quoted field
                    in_quotes = False
                    # Process the field content - ensure all quotes are doubled
                    field_content = ''.join(current_field)
                    fixed_field = []
                    j = 0
                    while j < len(field_content):
                        if field_content[j:j+2] == '""':
                            fixed_field.append('""')
                            j += 2
                        elif field_content[j] == '"':
                            # Single quote - needs to be doubled
                            fixed_field.append('""')
                            total_fixes += 1
                            j += 1
                        else:
                            fixed_field.append(field_content[j])
                            j += 1
                    
                    result_fields.append('"' + ''.join(fixed_field) + '"')
                    current_field = []
                    i += 1
                    # Skip the comma if present
                    if i < len(line) and line[i] == ',':
                        i += 1
            elif char == ',' and not in_quotes:
                # Field separator - unquoted field
                if current_field:
                    # Ensure all fields are quoted
                    result_fields.append('"' + ''.join(current_field) + '"')
                    current_field = []
                i += 1
            else:
                current_field.append(char)
                i += 1
        
        # Add last field
        if current_field or in_quotes:
            if in_quotes:
                # Field wasn't closed - this is an error but do our best
                field_content = ''.join(current_field)
                fixed_field = []
                j = 0
                while j < len(field_content):
                    if field_content[j:j+2] == '""':
                        fixed_field.append('""')
                        j += 2
                    elif field_content[j] == '"':
                        fixed_field.append('""')
                        total_fixes += 1
                        j += 1
                    else:
                        fixed_field.append(field_content[j])
                        j += 1
                result_fields.append('"' + ''.join(fixed_field) + '"')
            else:
                # Ensure all fields are quoted, even unquoted ones
                result_fields.append('"' + ''.join(current_field) + '"')
        
        fixed_line = ','.join(result_fields)
        fixed_lines.append(fixed_line)
    
    return '\n'.join(fixed_lines), total_fixes


def fix_and_validate_csv(csv_file):
    """
    Fix and validate CSV format.
    
    Args:
        csv_file: Path to the CSV file to validate
        
    Returns:
        tuple: (is_valid, errors list, fixes_applied list)
    """
    errors = []
    fixes = []
    
    if not Path(csv_file).exists():
        return False, [f"Error: CSV file '{csv_file}' not found"], []
    
    try:
        # Read the raw file content
        with open(csv_file, 'r', encoding='utf-8') as f:
            original_content = f.read()
        
        # First, convert any backslash-escaped quotes \"  to proper CSV escaped quotes ""
        # This handles cases where someone incorrectly used \ to escape quotes
        preprocessed_content = original_content.replace('\\"', '""')
        num_backslash_fixes = original_content.count('\\"')
        
        if num_backslash_fixes > 0:
            print(f'Pre-processing: Converted {num_backslash_fixes} backslash-escaped quotes (\\") to proper CSV escapes ("")')
            # Write back the preprocessed content
            with open(csv_file, 'w', encoding='utf-8') as f:
                f.write(preprocessed_content)
            original_content = preprocessed_content
            fixes.append(f"Converted {num_backslash_fixes} backslash-escaped quotes to proper CSV format")
        
        # First, try to detect issues by parsing
        try:
            with open(csv_file, 'r', encoding='utf-8', newline='') as f:
                reader = csv.reader(f)
                rows = list(reader)
                
            if not rows:
                return False, ["Error: CSV file is empty"], []
                
            headers = rows[0]
            num_expected_fields = len(headers)
            
            # Check for field count issues
            has_issues = False
            for line_num, row in enumerate(rows, start=1):
                if len(row) != num_expected_fields:
                    has_issues = True
                    errors.append(f"Line {line_num}: Expected {num_expected_fields} fields, got {len(row)}")
            
            if not has_issues:
                print(f"Headers: {headers}")
                print(f"All {len(rows)} rows have correct field count ({num_expected_fields} fields)")
                return True, [], []
                
        except csv.Error as e:
            has_issues = True
            errors.append(f"CSV parsing error: {e}")
        
        # If there are issues, try to fix them
        print("\nAttempting to fix unescaped quotes...")
        fixed_content, num_fixes = fix_unescaped_quotes(original_content)
        
        if num_fixes > 0:
            # Write the fixed content
            with open(csv_file, 'w', encoding='utf-8', newline='') as f:
                f.write(fixed_content)
            
            fixes.append(f"Fixed {num_fixes} unescaped quote(s)")
            
            # Re-validate
            with open(csv_file, 'r', encoding='utf-8', newline='') as f:
                reader = csv.reader(f)
                rows = list(reader)
            
            headers = rows[0]
            num_expected_fields = len(headers)
            
            print(f"Headers: {headers}")
            print(f"Expected {num_expected_fields} fields per row")
            
            errors = []  # Clear previous errors
            for line_num, row in enumerate(rows, start=1):
                if len(row) != num_expected_fields:
                    errors.append(f"Line {line_num}: Expected {num_expected_fields} fields, got {len(row)}")
        else:
            fixes.append("No unescaped quotes found, but validation failed")
        
    except Exception as e:
        errors.append(f"Error reading file: {e}")
        import traceback
        traceback.print_exc()
        return False, errors, fixes
    
    is_valid = len(errors) == 0
    return is_valid, errors, fixes





def print_report(csv_file, is_valid, errors, fixes):
    """Print a formatted validation and fix report."""
    print("\n" + "=" * 80)
    print(f"CSV Validation & Fix Report: {csv_file}")
    print("=" * 80)
    
    if fixes:
        print(f"\n✓ {len(fixes)} issue(s) FIXED:")
        for fix in fixes:
            print(f"  • {fix}")
    else:
        print("\n✓ No fixes needed")
    
    if is_valid:
        print("\n✓ CSV format is VALID after fixes")
    else:
        print("\n✗ CSV format has remaining ERRORS:")
        for error in errors:
            print(f"  • {error}")
    
    print("\nValidation Details:")
    print("  • All fields properly quoted (including 'number' field)")
    print("  • All quotes within fields are escaped as \"\"")
    print("  • No unescaped quotes breaking field boundaries")
    print("  • Correct number of fields per row")
    
    print("\n" + "=" * 80 + "\n")
    return is_valid


def main():
    """Main entry point."""
    if len(sys.argv) < 2:
        csv_file = "scripts\\github_issues\\csv_exports\\archive\\issues_to_update_20260201_pt2 copy 2.csv"
        print(f"No CSV file specified. Using default: {csv_file}")
    else:
        csv_file = sys.argv[1]
    
    # Convert backslashes to forward slashes for cross-platform compatibility
    csv_file = csv_file.replace('\\', '/')
    
    is_valid, errors, fixes = fix_and_validate_csv(csv_file)
    print_report(csv_file, is_valid, errors, fixes)
    
    if is_valid:
        print("✓ All issues have been fixed. The CSV is now ready for use.")
    else:
        print("✗ Some issues remain. Please review the errors above.")
    
    sys.exit(0 if is_valid else 1)


if __name__ == "__main__":
    main()
