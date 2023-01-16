import  enum

class Color(enum.Enum):
    # RGBA tuple
    RED = (1,0,0,1)
    REDGLASS = (0.8,0,0,0.8)
    GREEN = (0,1,0,1)
    BLUE = (0,0,1,1)
    BLUEGLASS = (0,0,0.8,0.6)
    GRAY = (0.8,0.8,0.8,1)
    GLASS = (0,0,0,0.2)
    LIGHT_GLASS = (1,1,1,0.2)
    BLACK_GLASS = (0,0,0,0.99)
    WHITE = (1,1,1,1)

class PrintColor:
    HEADER = '\033[95m'
    OKBLUE = '\033[94m'
    OKCYAN = '\033[96m'
    OKGREEN = '\033[92m'
    YELLOW = '\033[93m'
    FAIL = '\033[91m'
    ENDC = '\033[0m'
    BOLD = '\033[1m'
    UNDERLINE = '\033[4m'