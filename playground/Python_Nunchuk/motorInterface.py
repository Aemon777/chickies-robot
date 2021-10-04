
#State is defined as a 4-List carrying (pwmLeft, dirLeft, pwmRight, dirRight)
def drive(movspd, movdir, turnrat, turndir, curState):
    if movspd == 0:
        newState = swivel(movspd, movdir, turnrat, turndir)
    elif turnrat/movspd > 2:
        newState = pivot(movspd, movdir, turnrat, turndir)
    elif turnrat > 0:
        newState = curve(movspd, movdir, turnrat, turndir)
    else:
        newState = ([movspd, movdir, movspd, movdir])
    #Limit 10 PWM change per return
    
    if curState[1] != newState[1]:
        if newState[0] <= 10:
            newState[0] = 0
        else:
            newState[0] = newState[0] - 10
            newState[0] = not newState[0]
    else:
        if curState[0] - newState[0] > 10:
            newState[0] = curState[0] - 10
        elif newState[0] - curState[0] > 10:
            newState[0] = curState[0] + 10
    if curState[3] != newState[3]:
        if newState[2] <= 10:
            newState[2] = 0
        else:
            newState[2] = newState[2] - 10
            newState[2] = not newState[2]
    else:
        if curState[2] - newState[2] > 10:
            newState[2] = curState[2] - 10
        elif newState[2] - curState[2] > 10:
            newState[2] = curState[2] + 10

    
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
        pwmRight = movspd - turnrat/2
    else:
        pwmLeft = movspd - turnrat/2
        pwmRight = movspd
    return([pwmLeft, dirLeft, pwmRight, dirRight])
