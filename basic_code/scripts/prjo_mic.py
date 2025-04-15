import serial
import wave
import numpy as np

# 시리얼 포트 설정 (ESP32 포트 확인)
port = 'COM16'  # ESP32가 연결된 포트
baud_rate = 115200

# 오디오 파일 설정
output_file = 'recorded_audio.wav'
sample_rate = 8000  # ESP32와 동일한 샘플 속도

# 시리얼 데이터 읽기
print("Reading audio data from ESP32...")
with serial.Serial(port, baud_rate, timeout=10) as ser:
    audio_data = []
    while True:
        line = ser.readline().decode('utf-8').strip()
        if line == "Transmission complete!":
            break
        try:
            audio_data.append(int(line))
        except ValueError:
            continue

# 데이터 정규화
audio_array = np.array(audio_data)
audio_array = (audio_array - np.min(audio_array)) / (np.max(audio_array) - np.min(audio_array))  # 0~1로 정규화
audio_array = (audio_array * 65535 - 32768).astype(np.int16)  # 16비트 PCM

# WAV 파일로 저장
print(f"Saving audio to {output_file}...")
with wave.open(output_file, 'w') as wav_file:
    wav_file.setnchannels(1)  # 모노
    wav_file.setsampwidth(2)  # 16비트
    wav_file.setframerate(sample_rate)
    wav_file.writeframes(audio_array.tobytes())

print("Audio saved successfully!")
