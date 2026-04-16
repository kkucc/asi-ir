import time
import TimeTagger

CHANNEL = 5

def main():
    tagger = None
    try:
        tagger = TimeTagger.createTimeTagger()
        print(f"Connected: {tagger.getModel()}, S/N: {tagger.getSerial()}")
        
        tagger.setTriggerLevel(CHANNEL, 1.7)        
        rate = TimeTagger.Countrate(tagger, channels=[CHANNEL])
        
        print(f"\nReading channel {CHANNEL} (Press Ctrl+C to stop)\n")
        print("-" * 50)
        
        while True:
            time.sleep(0.2) 
            counts_hz = rate.getData()[0] 
            
            if counts_hz > 0:
                print(f"Channel {CHANNEL} | Frequency: {counts_hz:.2f} Hz      ")
            else:
                print(f"Channel {CHANNEL} | Frequency: 0.00 Hz", end='\r')

    except KeyboardInterrupt:
        print("\n\nKeyboard interrupt")
    except Exception as e:
        print(f"\nError: {e}")
    finally:
        if tagger is not None:
            TimeTagger.freeTimeTagger(tagger)
            print("Done.")

if __name__ == "__main__":
    main()