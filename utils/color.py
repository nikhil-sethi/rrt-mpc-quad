import  enum

class Color(enum.Enum):
    # RGBA tuple
    RED = (1,0,0,1)
    REDGLASS = (0.8,0,0,0.8)
    GREEN = (0,1,0,1)
    BLUE = (0,0,1,1)
    BLUEGLASS = (0,0,0.8,0.8)
    GRAY = (0.8,0.8,0.8,1)
    GLASS = (0.8,0.8,0.8,0.6)
    