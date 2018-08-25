import math
import time
from Util import *

from rlbot.agents.base_agent import BaseAgent, SimpleControllerState
from rlbot.utils.structures.game_data_struct import GameTickPacket

boosts = [
    [3584, 0],
    [-3584, 0],
    [3072, 4096],
    [3072, 4096]
    ]

def takeShot(agent):
    leftPost = Vector3([-sign(agent.team)*850, 5100*-sign(agent.team), 100])
    rightPost = Vector3([sign(agent.team)*850, 5100*-sign(agent.team), 100])
    centerGoal = Vector3([sign(agent.team)* math.cos(abs(agent.me.rotation.data[1]))*850,5100*-sign(agent.team),100])
    time = timeZ(agent.ball)

    bloc = future(agent.ball,time)
    goal_to_agent = (agent.me.location - centerGoal).normalize()
    goal_to_ball = (bloc - centerGoal).normalize()
    difference = goal_to_ball - goal_to_agent
    error = cap(abs(difference.data[0])+ abs(difference.data[1]),1,10)
    target_distance =cap( (100 + distance2D(bloc,agent.me)* (error**2))/1.95, 0,3000)
    target_location = bloc + Vector3([goal_to_ball.data[0]*target_distance, goal_to_ball.data[1]*target_distance,0])
    target_location.data[0] = cap(target_location.data[0],-4120,4120)

    target_local = toLocal(target_location, agent.me)
    angle_to_target = math.atan2(target_local.data[1],target_local.data[0])
    distance_to_target = distance2D(agent.me.location, target_location)
    speedCorrection = ((1+abs(angle_to_target))**2) * 300
    speed = 2300 - speedCorrection + cap((distance_to_target/16)**2, 0,speedCorrection)

    colorRed = cap(int( (speed/2300) * 255),0,255)
    colorBlue =cap(255-colorRed,0,255)

    agent.renderer.begin_rendering()
    agent.renderer.draw_line_3d(bloc.data, leftPost.data, agent.renderer.create_color(255,255,0,0))
    agent.renderer.draw_line_3d(bloc.data, rightPost.data, agent.renderer.create_color(255,0,255,0))

    agent.renderer.draw_line_3d(centerGoal.data,target_location.data, agent.renderer.create_color(255,colorRed,0,colorBlue))
    agent.renderer.draw_rect_3d(target_location.data, 10,10, True, agent.renderer.create_color(255,colorRed,0,colorBlue))
    agent.renderer.end_rendering()

    return target_location, speed
'''
    goal_to_agent = (agent.me.location - centerGoal).normalize()
    goal_to_ball = (agent.ball.location - centerGoal).normalize()
    difference = goal_to_ball - goal_to_agent
    error = cap(abs(difference.data[0])+ abs(difference.data[1]),1,10)
    target_distance =cap( (100 + distance2D(agent.ball,agent.me)* (error**2))/1.95, 0,3000)
    target_location = agent.ball.location + Vector3([goal_to_ball.data[0]*target_distance, goal_to_ball.data[1]*target_distance,0])
    target_location.data[0] = cap(target_location.data[0],-4120,4120)

    target_local = toLocal(target_location, agent.me)
    angle_to_target = math.atan2(target_local.data[1],target_local.data[0])
    distance_to_target = distance2D(agent.me.location, target_location)
    speedCorrection = ((1+abs(angle_to_target))**2) * 300
    speed = 2300 - speedCorrection + cap((distance_to_target/16)**2, 0,speedCorrection)

    colorRed = cap(int( (speed/2300) * 255),0,255)
    colorBlue =cap(255-colorRed,0,255)

    agent.renderer.begin_rendering()
    agent.renderer.draw_line_3d(agent.ball.location.data, leftPost.data, agent.renderer.create_color(255,255,0,0))
    agent.renderer.draw_line_3d(agent.ball.location.data, rightPost.data, agent.renderer.create_color(255,0,255,0))

    agent.renderer.draw_line_3d(centerGoal.data,target_location.data, agent.renderer.create_color(255,colorRed,0,colorBlue))
    agent.renderer.draw_rect_3d(target_location.data, 10,10, True, agent.renderer.create_color(255,colorRed,0,colorBlue))
    agent.renderer.end_rendering()

    return target_location, speed
'''
def drive(agent,data):
    controller = SimpleControllerState()
    target = toLocal(data[0],agent.me)
    speed = data[1]

    angle = math.atan2(target.data[1],target.data[0])

    currentSpeed = velocity2D(agent.me)
    controller.steer = steer(angle)

    if speed > currentSpeed:
        controller.throttle = 1.0
        if speed > 1400 and currentSpeed < 2200:
            controller.boost = True
    elif speed < currentSpeed:
        controller.throttle = -1.0
    return controller

class ThreeHourDream(BaseAgent):

    def initialize_agent(self):
        self.me= obj()
        self.ball = obj()

    def get_output(self, game: GameTickPacket) -> SimpleControllerState:

        self.preprocess(game)              
        return drive(self, takeShot(self))

    def preprocess(self, game):
        self.me.location.data = [game.game_cars[self.index].physics.location.x,game.game_cars[self.index].physics.location.y,game.game_cars[self.index].physics.location.z]
        self.me.velocity.data = [game.game_cars[self.index].physics.velocity.x,game.game_cars[self.index].physics.velocity.y,game.game_cars[self.index].physics.velocity.z]
        self.me.rotation.data = [game.game_cars[self.index].physics.rotation.pitch,game.game_cars[self.index].physics.rotation.yaw,game.game_cars[self.index].physics.rotation.roll]

        self.me.e = game.game_cars[not self.index].physics.rotation.yaw

        
        self.me.rvelocity.data = [game.game_cars[self.index].physics.angular_velocity.x,game.game_cars[self.index].physics.angular_velocity.y,game.game_cars[self.index].physics.angular_velocity.z]
        self.me.matrix = rotator_to_matrix(self.me)
        self.me.boost = game.game_cars[self.index].boost

        

        self.ball.location.data = [game.game_ball.physics.location.x,game.game_ball.physics.location.y,game.game_ball.physics.location.z]
        self.ball.velocity.data = [game.game_ball.physics.velocity.x,game.game_ball.physics.velocity.y,game.game_ball.physics.velocity.z]
        self.ball.rotation.data = [game.game_ball.physics.rotation.pitch,game.game_ball.physics.rotation.yaw,game.game_ball.physics.rotation.roll]
        self.ball.rvelocity.data = [game.game_ball.physics.angular_velocity.x,game.game_ball.physics.angular_velocity.y,game.game_ball.physics.angular_velocity.z]
        self.ball.localLocation = to_local(self.ball,self.me)
