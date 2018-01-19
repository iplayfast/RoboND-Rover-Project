import numpy as np

def move_toward(Rover,dist,ang,maxSpeed,followWall):
    if len(ang) >= Rover.stop_forward:  
        # If mode is forward, navigable terrain looks good 
        # and velocity is below max, then throttle 
        if Rover.vel < maxSpeed:
            # Set throttle value to throttle setting
            Rover.throttle = Rover.throttle_set
        else: # Else coast
            Rover.throttle = 0
        Rover.brake = 0
        offset = 0
        if followWall:
            offset = 0.8 * np.std(Rover.nav_angles)
        Rover.steer = offset + np.clip(np.mean(ang * 180 / np.pi), -15,15)
    elif len(ang) < Rover.stop_forward:
       # Set mode to "stop" and hit the brakes!
       Rover.throttle = 0
       # Set brake to stored brake value
       Rover.brake = Rover.brake_set
       Rover.steer = 0
       Rover.mode = 'stop'
    return Rover

def stop(Rover):
# If we're in stop mode but still moving keep braking
    if Rover.vel > 0.2:
        Rover.throttle = 0
        Rover.brake = Rover.brake_set
        Rover.steer = 0
    else:
        Rover.mode = 'stopped'
        Rover.stopAngle = Rover.yaw # steer might be better
    return Rover
 
def turn(Rover,ang):
    Rover.mode = 'turning'
    if round(Rover.yaw) == round(ang):
        Rover.mode = 'forward'
    else:
        Rover.throttle = 0
        Rover.brake = 0
        sign = 1
        if Rover.yaw > ang:
            sign = -1
        Rover.steer = np.clip((abs(Rover.yaw) - abs(ang))* sign,-15,15)
        print('turning from '+str(Rover.yaw) + ' to ' + str(ang) + ' steer ' + str(Rover.steer))
    return Rover


#if we are going from rocks present to non-rocks present or visaversa then
#modes change to equivilant
def updatemode(rocksHere):
    if rocksHere:
        switcher = {
                'forward': 'Rforward',
                'stop': 'Rstop',
                'turning': 'Rturning',
                'stopped': 'Rstopped',
                }
        return switcher(Rover.mode,Rover.mode)
    else:
        switcher = {
                'Rforward': 'forward',
                'Rstop': 'stop',
                'Rturning': 'turning',
                'Rstopped': 'stopped',
                }
        return switcher(Rover.mode,Rover.mode)
 
        
        
        
# This is where you can build a decision tree for determining throttle, brake and steer 
# commands based on the output of the perception_step() function
def decision_step(Rover):

    # Implement conditionals to decide what to do given perception data
    # Here you're all set up with some basic functionality but you'll need to
    # improve on this decision tree to do a good job of navigating autonomously!

    # Example:
    # Check if we have vision data to make decisions with

    if Rover.rock_angles is not None and len(Rover.rock_angles)>0:
        ang = np.mean(Rover.rock_angles * 180/np.pi)  
        print("Rock! angle " + str(ang) + ' dist ' +
                str(np.mean(Rover.rock_dists)))
        if Rover.mode=='stop' or round(Rover.yaw)==round(ang):
            if (np.mean(Rover.rock_dists)<10):
                Rover.mode = 'stop'
            else:
                Rover = move_toward(Rover,Rover.rock_dists,Rover.rock_angles,Rover.max_vel/2,False)
        elif Rover.mode=='forward':
            Rover = turn(Rover,ang)
        elif Rover.mode=='turning':
            Rover = turn(Rover,ang)

#        Rover = move_toward(Rover,Rover.rock_dists,Rover.rock_angles,Rover.max_vel/2,False)
    elif Rover.nav_angles is not None:
        
        # Check for Rover.mode status
        if Rover.mode == 'forward': 
            nSamples = 100
            Rover.avgspd -= Rover.avgspd / nSamples
            Rover.avgspd += Rover.vel / nSamples
            if Rover.avgspd < 1:  #we are stuck
                Rover.stopAngle = Rover.yaw + -180 + 30
                Rover.avgspd = nSamples * 10 # give it a chance
                Rover.mode = 'turning'
            else:
                Rover = move_toward(Rover,Rover.nav_dists,Rover.nav_angles,Rover.max_vel,True)
        # If we're already in "stop" mode then make different decisions
        elif Rover.mode == 'stop':
            Rover = stop(Rover)
        elif Rover.mode=='stopped':
            Rover = turn(Rover,(Rover.stopAngle +180) % 360)
        elif Rover.mode=='turning':
            Rover = turn(Rover,(Rover.stopAngle +180) % 360)

    # Just to make the rover do something 
    # even if no modifications have been made to the code
    else:
        Rover.throttle = Rover.throttle_set
        Rover.steer = 0
        Rover.brake = 0
        
    # If in a state where want to pickup a rock send pickup command
    if Rover.near_sample and Rover.vel == 0 and not Rover.picking_up:
        Rover.send_pickup = True
    print( 'mode ' + Rover.mode + ' angles ' + str(np.mean(Rover.nav_angles)) + 'dists ' +
            str(np.mean(Rover.nav_dists))+' rock angles ' +
            str(np.mean(Rover.rock_angles)) + ' dists ' +
            str(np.mean(Rover.rock_dists)))
    return Rover
'''
# This is where you can build a decision tree for determining throttle, brake and steer 
# commands based on the output of the perception_step() function
def decision_stepold(Rover):

    # Implement conditionals to decide what to do given perception data
    # Here you're all set up with some basic functionality but you'll need to
    # improve on this decision tree to do a good job of navigating autonomously!

    # Example:
    # Check if we have vision data to make decisions with
    if Rover.nav_angles is not None:
        # Check for Rover.mode status
        if Rover.mode == 'forward': 
            # Check the extent of navigable terrain
            if len(Rover.nav_angles) >= Rover.stop_forward:  
                # If mode is forward, navigable terrain looks good 
                # and velocity is below max, then throttle 
                if Rover.vel < Rover.max_vel:
                    # Set throttle value to throttle setting
                    Rover.throttle = Rover.throttle_set
                else: # Else coast
                    Rover.throttle = 0
                Rover.brake = 0
                offset = 0.8 * np.std(Rover.nav_angles)
                # Set steering to average angle clipped to the range +/- 15
                Rover.steer = offset + np.clip(np.mean(Rover.nav_angles * 180/np.pi), -15, 15)
                    
            # If there's a lack of navigable terrain pixels then go to 'stop' mode
            elif len(Rover.nav_angles) < Rover.stop_forward:
                    # Set mode to "stop" and hit the brakes!
                    Rover.throttle = 0
                    # Set brake to stored brake value
                    Rover.brake = Rover.brake_set
                    Rover.steer = 0
                    Rover.mode = 'stop'

        # If we're already in "stop" mode then make different decisions
        elif Rover.mode == 'stop':
            # If we're in stop mode but still moving keep braking
            if Rover.vel > 0.2:
                Rover.throttle = 0
                Rover.brake = Rover.brake_set
                Rover.steer = 0
            # If we're not moving (vel < 0.2) then do something else
            elif Rover.vel <= 0.2:
                # Now we're stopped and we have vision data to see if there's a path forward
                if len(Rover.nav_angles) < Rover.go_forward:
                    Rover.throttle = 0
                    # Release the brake to allow turning
                    Rover.brake = 0
                    # Turn range is +/- 15 degrees, when stopped the next line will induce 4-wheel turning
                    Rover.steer = -15 # Could be more clever here about which way to turn
                # If we're stopped but see sufficient navigable terrain in front then go!

                if len(Rover.nav_angles) >= Rover.go_forward:
                    # Set throttle back to stored value
                    Rover.throttle = Rover.throttle_set
                    # Release the brake
                    Rover.brake = 0
                    # Set steer to mean angle
                    Rover.steer = np.clip(np.mean(Rover.nav_angles * 180/np.pi), -15, 15)
                    # set steer to just right of mean angle so wall is followed
                    #Rover.steer = np.clip(np.mean(Rover.nav_angles * 180/np.pi), -20, 10)
                    Rover.mode = 'forward'
    # Just to make the rover do something 
    # even if no modifications have been made to the code
    else:
        Rover.throttle = Rover.throttle_set
        Rover.steer = 0
        Rover.brake = 0
        
    # If in a state where want to pickup a rock send pickup command
    if Rover.near_sample and Rover.vel == 0 and not Rover.picking_up:
        Rover.send_pickup = True
    
    return Rover
'''
