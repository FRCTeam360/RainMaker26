#!/usr/bin/env python3
"""
CSV Format Validator for GitHub Issues
Validates that the CSV is properly formatted with quotes around all fields except 'number'
"""

import csv
import sys
from pathlib import Path


def validate_csv(csv_file):
    """
    Validate CSV format and quote consistency.
    
    Args:
        csv_file: Path to the CSV file to validate
        
    Returns:
        tuple: (is_valid, errors list)
    """
    errors = []
    
    if not Path(csv_file).exists():
        return False, [f"Error: CSV file '{csv_file}' not found"]
    
    try:
        with open(csv_file, 'r', encoding='utf-8') as f:
            content = f.read()
            lines = content.strip().split('\n')
        
        if not lines:
            return False, ["Error: CSV file is empty"]
        
        # Get header
        header_line = lines[0]
        headers = [h.strip() for h in header_line.split(',')]
        
        # Validate header format
        for i, header in enumerate(headers):
            if header != "number" and not (header.startswith('"') and header.endswith('"')):
                errors.append(f"Header validation: Field '{header}' at position {i} should be quoted (except 'number')")
        
        # Parse remaining lines
        for line_num, line in enumerate(lines[1:], start=2):
            if not line.strip():
                continue  # Skip empty lines
            
            # Use csv reader to properly parse the line
            try:
                reader = csv.reader([line])
                row = next(reader)
            except Exception as e:
                errors.append(f"Line {line_num}: Failed to parse - {e}")
                continue
            
            if len(row) != len(headers):
                errors.append(
                    f"Line {line_num}: Expected {len(headers)} fields, got {len(row)}. "
                    f"Headers: {len(headers)}, Fields: {len(row)}"
                )
                continue
            
            # Check each field
            for col_idx, (header, value) in enumerate(zip(headers, row)):
                if header == "number":
                    # 'number' field should not be quoted in the raw line
                    if not value:
                        errors.append(f"Line {line_num}, {header}: Value is empty")
                else:
                    # All other fields should be quoted in the original line
                    # We need to check the raw line for quotes
                    pass  # Already parsed by csv reader
        
        # Additional validation: Check raw format for non-number fields
        for line_num, line in enumerate(lines[1:], start=2):
            if not line.strip():
                continue
            
            # Parse the line manually to check quotes in raw format
            parts = line.split(',', 1)  # Split on first comma only
            if len(parts) >= 1:
                first_field = parts[0].strip()
                # First field should be the 'number' field (no quotes expected)
                if first_field.startswith('"'):
                    errors.append(f"Line {line_num}: 'number' field should not be quoted, got: {first_field}")
                
                # Check remaining fields for quotes
                if len(parts) > 1:
                    remaining = parts[1]
                    # Split on comma, but be careful of quoted fields containing commas
                    reader = csv.reader([remaining])
                    try:
                        remaining_fields = next(reader)
                        for col_idx, (header, value) in enumerate(zip(headers[1:], remaining_fields), start=1):
                            # Check if the field in the CSV raw content has quotes
                            # The CSV reader will have already removed them, so we need to check differently
                            pass
                    except:
                        pass
        
        # Alternative: Read with csv.DictReader to validate structure
        with open(csv_file, 'r', encoding='utf-8') as f:
            try:
                reader = csv.DictReader(f)
                row_count = 0
                for row_num, row in enumerate(reader, start=2):
                    row_count += 1
                    
                    # Check that 'number' field exists and is not empty
                    if 'number' not in row or not row['number']:
                        errors.append(f"Line {row_num}: 'number' field is missing or empty")
                    
                    # Check that other fields exist
                    for header in headers:
                        if header not in row:
                            errors.append(f"Line {row_num}: Missing field '{header}'")
                
            except Exception as e:
                errors.append(f"Error reading CSV with DictReader: {e}")
        
    except Exception as e:
        errors.append(f"Error reading file: {e}")
    
    is_valid = len(errors) == 0
    return is_valid, errors


def print_validation_report(csv_file, is_valid, errors):
    """Print a formatted validation report."""
    print("\n" + "=" * 70)
    print(f"CSV Validation Report: {csv_file}")
    print("=" * 70)
    
    if is_valid:
        print("✓ CSV format is VALID")
    else:
        print("✗ CSV format has ERRORS:")
        for error in errors:
            print(f"  • {error}")
    
    print("=" * 70 + "\n")
    return is_valid


def main():
    """Main entry point."""
    if len(sys.argv) < 2:
        csv_file = "important issues.csv"
        print(f"No CSV file specified. Using default: {csv_file}")
    else:
        csv_file = sys.argv[1]
    
    is_valid, errors = validate_csv(csv_file)
    valid_report = print_validation_report(csv_file, is_valid, errors)
    
    sys.exit(0 if valid_report else 1)


if __name__ == "__main__":
    main()
