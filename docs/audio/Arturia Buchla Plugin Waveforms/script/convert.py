import numpy as np
from scipy.io import wavfile
from scipy.interpolate import interp1d
import os

def generate_daisy_header(wav_path, output_header="BuchlaWave.h", array_name="buchla_wave"):
    # 1. Load the wave file
    sample_rate, data = wavfile.read(wav_path)
    
    # Handle stereo files by taking the left channel
    if len(data.shape) > 1:
        data = data[:, 0]

    # 2. Resample to exactly 512 points
    current_indices = np.linspace(0, 1, len(data))
    new_indices = np.linspace(0, 1, 512)
    interpolator = interp1d(current_indices, data, kind='cubic')
    resampled = interpolator(new_indices)
    
    # 3. Normalize to -1.0 to 1.0 and convert to float32
    max_val = np.max(np.abs(resampled))
    if max_val > 0:
        resampled = resampled / max_val
    
    # 4. Format into C++ string
    header_content = [
        f"// Generated from {os.path.basename(wav_path)}",
        "#ifndef " + array_name.upper() + "_H",
        "#define " + array_name.upper() + "_H",
        "",
        f"const float {array_name}[512] = {{"
    ]
    
    # Add values 8 per line for readability
    for i in range(0, 512, 8):
        line = "    " + ", ".join([f"{v:.6f}f" for v in resampled[i:i+8]]) + ","
        header_content.append(line)
        
    header_content.append("};")
    header_content.append("")
    header_content.append("#endif")
    
    # 5. Write to file
    with open(output_header, "w") as f:
        f.write("\n".join(header_content))
    
    print(f"Success! Created {output_header} with array '{array_name}'")

# Change 'my_sample.wav' to your actual file name
generate_daisy_header('buchla_triangle_1cyc.wav', 'BuchlaTriangle.h', 'buchlaTriangle_512')