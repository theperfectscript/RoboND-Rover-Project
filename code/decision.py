import numpy as np
import math
import sys

# FSM

class StateMachine:
    def __init__(self, initial_state):
        print('created state machine with initial state {}'.format(initial_state))
        self.current_state = initial_state
        self.current_state.run('none')

# Template method:
    def run(self, input):
        self.current_state = self.current_state.next(input)
        self.current_state.run(input)


class State:

    def run(self,input):
        raise NotImplementedError()
    def next(self, input):
        raise NotImplementedError()
    def vec_dist(self,v1, v2):
        return  math.hypot(v2[0] - v1[0], v2[1] - v1[1])
    def angle(self,p1,p2):
        return math.degrees(math.atan2(p2[1] - p1[1],p2[0] - p1[0]))

class Rover_idle(State):
    def run(self,input=None):
        print('idling...')
        return input

    def next(self, input):
        print('Idle next with input {}'.format(input))
        if input is None:
            return self
        # idle until rover has sight
        if input.nav_angles is not None:
            print('got signal, start crawling...')
            return Rover_crawl_left_wall()

        return self

class Rover_stop(State):
    def __init__(self):
        self.t_counter = 0

    def run(self,input):
        print('stopping ...')

        input.throttle = 0
        input.brake = 1

        if abs(input.vel) < 0.03:
            self.t_counter += 1
        else:
            self.t_counter = 0

        return input

    def next(self, input):

         # stop if near rock
        if input.sample_distance < 60 and input.sample_angle >  0:
            return Rover_approach_sample()

        if not input.obstacle_infront:
            return Rover_crawl_left_wall()

        if self.t_counter > 90:
            self.t_counter = 0
            return Rover_setback()

        if input.obstacle_infront:
            return Rover_steer_right()

        if input.wall_infront:
            return self


        return Rover_crawl_left_wall()

class Rover_steer_right(State):

    def __init__(self):
        print('steer right')
        self.t_counter = 0
        self.ang = -1


    def run(self,input):
        self.ang = max(-15, self.ang -0.5)
        input.steer = self.ang
        input.brake = 0
        input.throttle = max(input.throttle-.1, 0)

        if input.wall_infront:
            self.brake = 1
        else:
            self.brake = 0

        if abs(input.vel) < 0.03:
            self.t_counter += 1
        else:
            self.t_counter = 0

        if input.vel < -0.1:
            input.throttle = 0.5



    def next(self, input):

         # stop if near rock
        if input.sample_distance < 40 and input.sample_angle >  0:
            return Rover_approach_sample()

        if not input.obstacle_infront:
            return Rover_crawl_left_wall()

        if self.t_counter > 120:
            self.t_counter = 0
            return Rover_setback()

        return self

class Rover_collect_sample(State):
    def run(self, input):
        input.brake = 10
        input.send_pickup = True
        if not input.sees_sample:
            input.send_pickup = False

    def next(self, input):
        if not input.sees_sample and not input.picking_up:
            return Rover_crawl_left_wall()
        return self


class Rover_approach_sample(State):

    def __init__(self):
        self.stalled_counter = 0
        print('approaching sample...')

    def run(self, input):
        # print('approaching sample...{}'.format(self.stalled_counter))
        # curved approach to avoid stucking if rock is in left turn

        if input.sample_distance > 10 and (math.degrees(input.sample_angle) < 20):
            input.steer = -15
        else:
            input.steer = math.degrees(input.sample_angle)

        if abs(input.vel) <= 0.03:
            self.stalled_counter += 1

        else:
            self.stalled_counter = 0

        if input.vel > 0.5:
            input.brake = 1 - ( min(100, input.sample_distance)* 0.01 )
        else:
            input.brake = 0
            input.throttle = .2

    def next(self, input):

        if input.near_sample:
            return Rover_collect_sample()

        if self.stalled_counter >= 60:
            self.stalled_counter = 0
            return Rover_setback()

        if not input.sees_sample:
            return Rover_crawl_left_wall()

        return self

class Rover_return_home(State):

    def __init__(self):
        self.stalled_counter = 0
        self.start_position = None
        print('Returning home')

    def run(self,input):

        # print('Returning home, distance {}, angle {}'.format(self.vec_dist(input.pos, input.start_pos), self.angle(input.pos, input.start_pos)))
        self.steer = np.clip(self.angle(input.pos, input.start_pos), -15, 15)
        dist = self.vec_dist(input.pos, input.start_pos)

        if input.vel > dist:
            input.brake = input.vel - dist * 0.01

        self.throttle = np.clip(dist * 0.1, -1, 1)

        if input.vel < 0.03:
            self.stalled_counter += 1
        else:
            self.stalled_counter = 0

        return input

    def next(self, input):
        if self.stalled_counter > 60:
            return Rover_setback()

        if self.vec_dist(input.pos, input.start_pos) < 5:
            return Rover_done()
        return self

class Rover_done(State):

    def run(self, input):
        input.brake = 2
        print('Returned home, mapped {}% with {} fidelity in {} sec.'.format(
            input.percent_mapped,
            input.fidelity,
            input.total_time
            ))
        print('Collected {} out of {} samples on the way.'.format(
            input.samples_collected,
            input.samples_located
        ))
        print('Shutting down, goodbye...')


    def next(self, input):
        return self

class Rover_crawl_left_wall(State):

    def __init__(self):

        print('crawl wall... ')
        self.stalled_counter = 0
        self.max_speed = 3.5


    def run(self,input):
        if input.start_pos == None:
            input.start_pos = input.pos


        input.brake = 0

        # left steering bias
        mean_angle = np.mean(input.nav_angles)
        input.steer = np.clip( math.degrees(mean_angle) + 11, -15, 15)


        dynamic_max_speed = self.max_speed - max(0, 1000 - len(input.nav_angles)) * 0.001
        # dynamic_max_speed -= abs((input.steer-3)/10)

        if input.vel > dynamic_max_speed:
            input.brake = 1

        if input.vel < dynamic_max_speed:
            input.throttle = min( 0.7, (dynamic_max_speed - input.vel)*0.75)
        else:
            input.throttle = 0

        if input.vel > dynamic_max_speed:
            input.brake = 0.5

        if abs(input.vel) <= 0.15:
            self.stalled_counter += 1

        else:
            self.stalled_counter = 0

        return input

    def next(self, input):

        if input.percent_mapped > 80 and self.vec_dist(input.start_pos, input.pos) < 10:
            return Rover_return_home()

        if self.stalled_counter >= 60:
            self.stalled_counter = 0
            return Rover_setback()

        # idle if no sight
        if input.nav_angles is None:
            return Rover_idle()

        # stop if not enough navigable space in front
        if input.obstacle_infront:
            return Rover_stop()

        # stop if near rock
        if input.sample_distance < 60 and input.sample_angle >  -0.1*np.pi:
            return Rover_approach_sample()

        return self

class Rover_quicksand(State):

    def __init__(self):

        print('Quicksand, spin... ')
        self.t_counter = 0
        self.start_position = None

    def run(self, input):

        input.brake = 0
        input.steer = -15
        input.throttle = 0

        self.t_counter += 1

        return input

    def next(self, input):

        if self.t_counter > 150:
            return Rover_crawl_left_wall()
        return self




class Rover_setback(State):

    def __init__(self):
        print('stalled, set back... ')
        self.t_counter = 0
        self.start_position = None

    def run(self, input):

        if self.start_position == None:
            self.start_position = input.pos

        input.brake = 0
        input.steer = 0
        input.throttle = -2

        self.t_counter += 1

        return input

    def next(self, input):

        if self.t_counter > 25:
            print(self.vec_dist(input.pos, self.start_position))
            if self.vec_dist(input.pos, self.start_position) < 0.1:
                return Rover_quicksand()
            else:
                return Rover_crawl_left_wall()
        return self

class Decision:
    # This is where you can build a decision tree for determining throttle, brake and steer
    # commands based on the output of the perception_step() function
    def __init__(self):
        self.state_machine = StateMachine(Rover_idle())

    def decision_step(self,Rover):


        # Implement conditionals to decide what to do given perception data
        # Here you're all set up with some basic functionality but you'll need to
        # improve on this decision tree to do a good job of navigating autonomously!
        self.state_machine.run(Rover)
        return Rover

