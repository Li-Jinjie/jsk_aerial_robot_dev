import subprocess
import sys
import os


def convert_mp4_to_wav(input_mp4, output_wav):
    try:
        subprocess.run(["ffmpeg", "-version"], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    except FileNotFoundError:
        print("Error: ffmpeg not installed")
        return

    os.makedirs(os.path.dirname(output_wav) or ".", exist_ok=True)

    cmd = ["ffmpeg", "-i", input_mp4, "-ac", "1", "-ar", "44100", output_wav, "-y"]

    subprocess.run(cmd, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)


if __name__ == "__main__":
    if len(sys.argv) != 3:
        print("python3 convert_mp4_to_wav.py input.mp4 output.wav")
        sys.exit(1)

    convert_mp4_to_wav(sys.argv[1], sys.argv[2])
