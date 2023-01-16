def obj(z,current_target):
   return (100.0*(z[4]-current_target[0])**2 + 100.0*(z[5]-current_target[1])**2 + 100.0*(z[6]-current_target[2])**2 +
            10*(z[0]/20000)**2 + 10*(z[1]/20000)**2 + 10*(z[2]/20000)**2 + 10*(z[3]/20000)**2)

def objN(z,current_target):
   return (200.0*(z[4]-current_target[0])**2 + 200.0*(z[5]-current_target[1])**2 + 200.0*(z[6]-current_target[2])**2 +
        20*(z[0]/20000)**2 + 20*(z[1]/20000)**2 + 20*(z[2]/20000)**2 + 20*(z[3]/20000)**2)