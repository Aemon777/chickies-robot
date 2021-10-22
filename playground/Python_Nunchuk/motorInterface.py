
#State is defined as a 4-List carrying (pwmLeft, dirLeft, pwmRight, dirRight)
def drive(movspd, movdir, turnrat, turndir, curState):
    if turnrat != 0:
        if movspd == 0:
            newState = swivel(movspd, movdir, turnrat, turndir)
        elif turnrat/movspd > 2:
            newState = pivot(movspd, movdir, turnrat, turndir)
        else:
           newState = curve(movspd, movdir, turnrat, turndir)
    else:
        newState = ([movspd, movdir, movspd, movdir])
    #Limit (curState[0]/15 + 1) PWM change per return
    if curState[1] != newState[1]:
        if curState[0] <= (curState[0]/15 + 1):
            newState[0] = abs(curState[0]-(curState[0]/15 + 1))
        else:
            newState[0] = curState[0] - (curState[0]/15 + 1)
            newState[1] = curState[1]
    else:
        if curState[0] - newState[0] > (curState[0]/15 + 1):
            newState[0] = curState[0] - (curState[0]/15 + 1)
        elif newState[0] - curState[0] > (curState[0]/15 + 1):
            newState[0] = curState[0] + (curState[0]/15 + 1)
    if curState[3] != newState[3]:
        if curState[2] <= (curState[2]/15 + 1):
            newState[2] = abs(curState[2]-(curState[2]/15 + 1))
        else:
            newState[2] = curState[2] - (curState[2]/15 + 1)
            newState[3] = curState[3]
    else:
        if curState[2] - newState[2] > (curState[2]/15 + 1):
            newState[2] = curState[2] - (curState[2]/15 + 1)
        elif newState[2] - curState[2] > (curState[2]/15 + 1):
            newState[2] = curState[2] + (curState[2]/15 + 1)
    newState[0] = int(newState[0])
    return (newState)

def swivel(movspd, movdir, turnrat, turndir):
    pwmLeft = turnrat
    pwmRight = turnrat
    if turndir == 'right':
        dirLeft = movdir
        dirRight = not movdir
    else:
        dirLeft = not movdir
        dirRight = movdir
    return ([pwmLeft, dirLeft, pwmRight, dirRight])

        

def pivot(movspd, movdir, turnrat, turndir):
    dirLeft = movdir
    dirRight = movdir
    if turndir == 'right':
        pwmLeft = turnrat
        pwmRight = 0
    else:
        pwmLeft = 0
        pwmRight = turnrat
    return([pwmLeft, dirLeft, pwmRight, dirRight])


def curve(movspd, movdir, turnrat, turndir):
    dirLeft = movdir
    dirRight = movdir
    if turndir == 'right':
        pwmLeft = movspd
        pwmRight = int(movspd - turnrat/2)
    else:
        pwmLeft = int(movspd - turnrat/2)
        pwmRight = movspd
    return([pwmLeft, dirLeft, pwmRight, dirRight])
