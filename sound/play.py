import pygame

# pygame 초기화
pygame.mixer.init()

# MP3 파일 경로
mp3_file = 'C:/Users/qutei/sound/man/1.mp3'
mp3_file1 = 'C:/Users/qutei/sound/man/2.mp3'

# MP3 파일 로드
pygame.mixer.music.load(mp3_file)

# 재생
pygame.mixer.music.play()

# 음악이 재생되는 동안 프로그램이 종료되지 않도록 대기
while pygame.mixer.music.get_busy(): #play 중이면
    pygame.time.Clock().tick(10) #루프가 10초에 한 번씩 돌게 함함
    
# MP3 파일 로드
pygame.mixer.music.load(mp3_file1)

# 재생
pygame.mixer.music.play()

# 음악이 재생되는 동안 프로그램이 종료되지 않도록 대기
while pygame.mixer.music.get_busy(): #play 중이면
    pygame.time.Clock().tick(10) #루프가 10초에 한 번씩 돌게 함함