import time
import TimeTagger

CHANNEL = 5

def main():
    tagger = None
    try:
        tagger = TimeTagger.createTimeTagger()
        print(f"Conneccted: {tagger.getModel()}, S/N: {tagger.getSerial()}")
        
        # уровень триггера на 0.5 Вольт?
        tagger.setTriggerLevel(CHANNEL, 1.7)
        print(f"{CHANNEL} 0.5?")
        rate = TimeTagger.Countrate(tagger, channels=[CHANNEL])
        
        print(f"\n Reading {CHANNEL}\n")
        print("-" * 50)
        
        while True:
            time.sleep(0.2) 
            ounts_hz = rate.getData()[0]
            if counts_hz > 0:
                print(f" {CHANNEL} | Частота: {counts_hz:.2f} Гц")
            else:
                print(f"{CHANNEL} | Частота: 0.00 Гц", end='\r')

    except KeyboardInterrupt:
        print("\n\nkeynord interapt")
    except Exception as e:
        print(f"\n {e}")
    finally:
        if tagger is not None:
            TimeTagger.freeTimeTagger(tagger)
            print("Done.")

if __name__ == "__main__":
    main()

#     Conneccted: Time Tagger Ultra, S/N: 22300011M7
# 5 0.5?

#  Reading 5

# --------------------------------------------------

#  name 'counts_hz' is not defined
# Done.
# PS C:\Users\QE LAB\Documents\Visual Studio 2022\vscode> 

