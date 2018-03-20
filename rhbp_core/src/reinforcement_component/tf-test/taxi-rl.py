import gym
import numpy as np
import random
import tensorflow as tf
import matplotlib.pyplot as plt

env = gym.make('Taxi-v2')
tf.reset_default_graph()


s = env.reset()
print(s)
env.render()

a,b,c,d=env.step(env.action_space.sample())
print(a,b,c,d)

print(env.observation_space)

print(env.action_space)


#These lines establish the feed-forward part of the network used to choose actions


num_input = 500
n_hidden_1 = 500
n_hidden_2 = 500
n_hidden_3 = 500
num_classes = 6

inputs1 = tf.placeholder(shape=[1,num_input],dtype=tf.float32,name="input")
input_layer=tf.Variable(tf.random_uniform([num_input, num_classes]),name="weights")
weights = {
    'w1': tf.Variable(tf.random_uniform([num_input, n_hidden_1])),
    'w2': tf.Variable(tf.random_uniform([n_hidden_1, n_hidden_2])),
    'w3': tf.Variable(tf.random_uniform([n_hidden_2, n_hidden_3])),
'w4': tf.Variable(tf.random_uniform([n_hidden_2, n_hidden_3])),
'w5': tf.Variable(tf.random_uniform([n_hidden_2, n_hidden_3])),
    'wout': tf.Variable(tf.random_uniform([n_hidden_3, num_classes]))
}

biases = {
    'b1': tf.Variable(tf.random_uniform([n_hidden_1])),
    'b2': tf.Variable(tf.random_uniform([n_hidden_2])),
    'b3': tf.Variable(tf.random_uniform([n_hidden_3])),
'b4': tf.Variable(tf.random_uniform([n_hidden_3])),
'b5': tf.Variable(tf.random_uniform([n_hidden_3])),
    'bout': tf.Variable(tf.random_uniform([num_classes]))
}

#W1 = tf.Variable(tf.random_uniform([16,32],0,0.01))

#W2 = tf.Variable(tf.random_uniform([32,32],0,0.01))
#W3 = tf.Variable(tf.random_uniform([32,16],0,0.01))
#Wout = tf.Variable(tf.random_uniform([16,4],0,0.01))

l1 = tf.add(tf.matmul(inputs1,weights["w1"]),biases["b1"])
l2 = tf.add(tf.matmul(l1,weights["w2"]),biases["b2"])
l3 = tf.add(tf.matmul(l2,weights["w3"]),biases["b3"])
l4 = tf.add(tf.matmul(l3,weights["w4"]),biases["b4"])
l5 = tf.add(tf.matmul(l4,weights["w5"]),biases["b5"])
#Qout = tf.add(tf.matmul(l5,weights["wout"]),biases["bout"])
Qout = tf.matmul(inputs1,input_layer,name="Qout")
#l1 = tf.matmul(inputs1,weights["w1"])
#l2 = tf.matmul(l1,weights["w2"])
#l3 = tf.matmul(l2,weights["w3"])
#Qout = tf.matmul(l2,weights["wout"])
#W = tf.Variable(tf.random_uniform([16,4],0,0.01))
#Qout= tf.matmul(inputs1,W)
predict = tf.argmax(Qout,1,name="predict")

#Below we obtain the loss by taking the sum of squares difference between the target and prediction Q values.
nextQ = tf.placeholder(shape=[1,num_classes],dtype=tf.float32,name="nextQ")
loss = tf.reduce_sum(tf.square(nextQ - Qout),name="loss")
trainer = tf.train.AdamOptimizer(learning_rate=0.1,name="trainer")
updateModel = trainer.minimize(loss,name="updateModel")

init = tf.global_variables_initializer()

# Set learning parameters
y = .99
e = 0.1
num_episodes = 20000
#create lists to contain total rewards and steps per episode
jList = []
rList = []

saver = tf.train.Saver()

#Run the operation by feeding input

#Prints 24 which is sum of (w1+w2)*b1



with tf.Session() as sess:
    sess.run(init)
    j = 0
    not_finished = True
    while not_finished:
        #Reset environment and get first new observation
        s = env.reset()
        rAll = 0
        d = False
        #j = 0
        #The Q-Network
        while True:
            j+=1
            #Choose an action by greedily (with e chance of random action) from the Q-network
            a,allQ = sess.run([predict,Qout],feed_dict={inputs1:np.identity(num_input)[s:s+1]})

            if np.random.rand(1) < e:
                a[0] = env.action_space.sample()
            #Get new state and reward from environment
            s1,r,d,_ = env.step(a[0])

            #Obtain the Q' values by feeding the new state through our network
            Q1 = sess.run(Qout,feed_dict={inputs1:np.identity(num_input)[s1:s1+1]})
            #Obtain maxQ' and set our target value for chosen action.
            maxQ1 = np.max(Q1)
            targetQ = allQ
            targetQ[0,a[0]] = r + y*maxQ1
            #Train our network using target and predicted Q values
            updated_model = sess.run([updateModel,weights["wout"]],feed_dict={inputs1:np.identity(num_input)[s:s+1],nextQ:targetQ})
            #rAll += r
            s = s1
            e = 1. / ((j / 50) + 10)
            jList.append(j)
            rList.append(r)
            #if r == 20:
            #    print("successful ", d)
            if d == True:
                #Reduce chance of random action as we train the model.

                #if r == 20 :
                #   print("succesfull transport")
                #print(j,r)
                s = env.reset()
                #break


            if j != 0 and j%1000 ==0:
                print "average reward after "+str(j)+" runs: " + str(sum(rList) / j)
                print "average reward between "+str(j-100)+" and "+str(j)+ "runs: " + str(sum(rList[j-100:j]) / 100) + "%"
            if sum(rList[j-100:j]) / 100 >9.7:
                print "average reward after " + str(j) + " runs: " + str(sum(rList) / j)
                print "average reward between " + str(j - 100) + " and " + str(j) + "runs: " + str(
                    sum(rList[j - 100:j]) / 100) + "%"
                print("finished after "+str(j))
                not_finished = False
                break
            if j >= 40000:
                saver.save(sess, 'tmp/my_taxi_test_model', global_step=1000)
                print("final")
                not_finished = False
                break
#Now, save the graph
    saver.save(sess, 'tmp/my_taxi_test_model',global_step=1000)
    print "Percent of succesful episodes: " + str(sum(rList)/num_episodes) + "%"