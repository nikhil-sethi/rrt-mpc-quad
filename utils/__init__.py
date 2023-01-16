from .color import PrintColor

def printRed(string): print("\033[91m {}\033[00m" .format(string))
def printC(string, color=PrintColor.BOLD+PrintColor.OKCYAN): print(color+string+PrintColor.ENDC)