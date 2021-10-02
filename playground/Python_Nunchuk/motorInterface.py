

def drive(movspd, movdir, turnrat, turndir):
    if turnrat > 85:
        return swivel(movspd, movdir, turnrat, turndir)
    elif turnrat > 75:
        return pivot(movspd, movdir, turnrat, turndir)
    elif turnrat > 0:
        return curve(movspd, movdir, turnrat, turndir)
    else:
        dirA = movdir
        dirB = movdir
        pwmA = movspd
        pwmB = movspd
    return ((pwmA, dirA, pwmB, dirB))

def swivel(movspd, movdir, turnrat, turndir):
    pwmA = turnrat
    pwmB = turnrat
    if turndir == 'right':
        dirA = movdir
        dirB = not movdir
    else:
        dirA = not movdir
        dirB = movdir
    return ((pwmA, dirA, pwmB, dirB))

        

def pivot(movspd, movdir, turnrat, turndir):
    dirA = movdir
    dirB = movdir
    if turndir == 'right':
        pwmA = turnrat
        pwmB = 0
    else:
        pwmA = turnrat
        pwmB = 0
    return((pwmA, dirA, pwmB, dirB))


def curve(movspd, movdir, turnrat, turndir):
    dirA = movdir
    dirB = movdir
    if turndir == 'right':
        pwmA = movspd
        pwmB = movspd - turnrat
    else:
        pwmA = movspd - turnrat
        pwmB = movspd
    return((pwmA, dirA, pwmB, dirB))
