from pygame import mixer 
import time
# 音频初始化
mixer.init()
# 加载音频文件路径 (路径必须真实存在，音频文件格式支持mp3/ogg等格式)
# 件名称/路径可以出现中文
mixer.music.load('./sounds/take off.mp3')
# mixer.music.play()
# time.sleep(5)
# mixer.music.stop()
mixer.music.play()
while mixer.music.get_busy():
    time.sleep