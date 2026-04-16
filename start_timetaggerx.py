import time
import numpy as np

try:
    import TimeTagger
except ImportError:
    exit()

tagger = None
try:
    print("Подключаемся к Time Tagger...")
    tagger = TimeTagger.createTimeTagger()
    print(f"{tagger.getModel()}, {tagger.getSerial()}")

    tagger.setTestSignal(1, True)
    print("Inside test signal, channel 1")

    # количество событий на канале 1.
    countrate = TimeTagger.Countrate(tagger=tagger, channels=[1])
    print("Измеритель 'Countrate' создан. Начинаем сбор данных...")
    time.sleep(0.5)

    print("-" * 40)
    for i in range(5):
        time.sleep(1) 
        
        data = countrate.getData()
        current_rate_hz = data[0]
        
        print(f"Секунда {i+1}: Текущая частота = {current_rate_hz / 1000:.2f} кГц")

except Exception as e:
    print(f"\n{e}")

finally:
    if tagger:
        tagger.setTestSignal(1, False)
        TimeTagger.freeTimeTagger(tagger)
        print("Done.")