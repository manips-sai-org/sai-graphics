import re

# File paths
input_file = 'muscles_modified.xml'
output_file = 'muscles_fixed.xml'

with open(input_file, 'r') as f:
    lines = f.readlines()

# Regex to match the three coordinates inside a <point> tag
# Example: <point> 0.0010, -0.0231, 0.1496 </point>
point_pattern = re.compile(r'(<point>\s*[^,]+,\s*[^,]+,\s*)([^<]+)(\s*</point>)')

for i in range(len(lines)):
    # Look for the right_lower_leg linkName
    if '<linkName>right_lower_leg</linkName>' in lines[i]:
        # The next line should be the point data
        if i + 1 < len(lines) and '<point>' in lines[i+1]:
            match = point_pattern.search(lines[i+1])
            if match:
                prefix = match.group(1)   # Everything up to the last comma (including the comma and spaces)
                z_str = match.group(2)    # The Z value string
                suffix = match.group(3)   # The closing </point> tag

                try:
                    z_val = float(z_str)
                    # If the Z value is strictly positive, make it negative
                    if z_val > 0:
                        # Strip any whitespace to append the minus sign cleanly, then restore it
                        z_new_str = "-" + z_str.lstrip() 
                        # Reconstruct the line
                        lines[i+1] = lines[i+1][:match.start()] + prefix + z_new_str + suffix + lines[i+1][match.end():]
                except ValueError:
                    pass # Failsafe in case of weird formatting

# Write the modifications out to a new file
with open(output_file, 'w') as f:
    f.writelines(lines)

print(f"Processing complete! Saved to {output_file}.")