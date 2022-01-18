import setting
import dau1
import dau2
if __name__ == "__main__":
    setting.init()
    print(setting.s)
    print('After mother')
    setting.s = 'mother'
    print(setting.s)
    dau1.dau1call()
    print(setting.s)
    dau2.dau2call()
    print(setting.s)
 
