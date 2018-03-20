import time, random, math
import numpy as np
import gym




def updateQTable(prevState, prevAction, reward, curState):
    q_table[prevState][prevAction] += LEARNING_RATE * (
    reward + DISCOUNT * max(q_table[curState]) - q_table[prevState][prevAction])


def getAction(curState):
    if random.random() < EPSILON:
        return env.action_space.sample()
    else:
        return np.argmax(q_table[curState])


GAME = 'Taxi-v2'
env = gym.make(GAME)
env.seed(1095)

RECORD = None
MAX_EPISODES = 100001
#MAX_STEPS = env.spec.timestep_limit  # 100 for FrozenLake v0
MAX_STEPS = 200
EPSILON = 1
DISCOUNT = 0.99
LEARNING_RATE = 0.01

in_dimen = env.observation_space.n
out_dimen = env.action_space.n
obsMin = 0
obsMax = env.observation_space.n
actionMin = 0
actionMax = env.action_space.n
q_table = np.zeros((in_dimen, out_dimen))

#env.monitor.start('Artificial Intelligence/' + GAME, force=True, video_callable=RECORD)

print("\nObservation\n--------------------------------")
print("Shape :", in_dimen, " | High :", obsMax, " | Low :", obsMin)
print("\nAction\n--------------------------------")
print("Shape :", out_dimen, " | High :", actionMax, " | Low :", actionMin, "\n")

totalreward = 0

for episode in range(MAX_EPISODES):

    if episode % 1000 == 0:
        print("Episode =", episode, " |  Avg Reward =", totalreward / 1000)
        totalreward = 0

    EPSILON -= 0.001
    curState = env.reset()
    for step in range(MAX_STEPS):
        # env.render()
        prevState = curState
        action = getAction(curState)
        curState, reward, done, info = env.step(action)
        totalreward += reward
        updateQTable(prevState, action, reward, curState)
        if done:
            break

#env.monitor.close()