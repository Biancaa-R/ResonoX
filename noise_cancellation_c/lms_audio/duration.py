import wave
import contextlib
fname = r'D:\Downloads\noise_cancellation_c\lms_audio\noisy_audio.wav'
with contextlib.closing(wave.open(fname,'r')) as f:
    frames = f.getnframes()
    rate = f.getframerate()
    duration = frames / float(rate)
    print(duration)

import wave

def get_wav_info(filename):
    with wave.open(filename, 'r') as f:
        frames = f.getnframes()
        rate = f.getframerate()
        duration = frames / float(rate)
        return duration, frames, rate

noisy_info = get_wav_info(r'D:\Downloads\noise_cancellation_c\lms_audio\noisy.wav')
white_info = get_wav_info(r'D:\Downloads\noise_cancellation_c\lms_audio\white.wav')

print(f"Noisy WAV: {noisy_info}")
print(f"White Noise WAV: {white_info}")
