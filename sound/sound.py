
#gtts : 원하는 음성 tts 생성 및 저장
#pygame : 음성 재생
#pip install gtts, pygame

from gtts import gTTS
import pygame

# 변환할 텍스트
text = "사용자의 위치를 재탐색 중입니다."

#text = "안녕하세요. 저는 휠체어 사용자의 쇼핑을 도와주는 쇼핑카트 따르미예요. 시스템 준비 중이니 잠시만 기다려 주세요."
#text = "저는 지금 고객님의 쇼핑을 도와주는 중이에요 전방 카메라를 가리지 않게 부탁 드려요"
#text = "따르미 쇼핑을 도와줄 준비 완료되었어요 "
#text = "사용자의 위치를 재탐색 중입니다."
# gTTS 객체 생성하여 lang에 en으로 설정
tts = gTTS(text=text, lang='ko')

# 음성 파일 저장
tts.save("4.mp3")

print("TTS 음성 파일 생성 완료")
