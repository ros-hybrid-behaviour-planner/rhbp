import tensorflow as tf
import gym
import numpy as np
import random
import matplotlib.pyplot as plt
with tf.Session() as sess:
    #init = tf.global_variables_initializer()

    #sess.run(init)
    #First let's load meta graph and restore weights
    saver = tf.train.import_meta_graph('tmp/my_taxi_test_model-1000.meta')
    saver.restore(sess,tf.train.latest_checkpoint('./tmp'))

    print(sess.run("weights:0"))

    env = gym.make('Taxi-v2')


    #These lines establish the feed-forward part of the network used to choose actions

    num_input = 500

    #W2 = tf.Variable(tf.random_uniform([32,32],0,0.01))
    #W3 = tf.Variable(tf.random_uniform([32,16],0,0.01))
    #Wout = tf.Variable(tf.random_uniform([16,4],0,0.01))

    #Qout = tf.add(tf.matmul(l5,weights["wout"]),biases["bout"])
    #init = tf.global_variables_initializer()

    #sess.run(init)
    #print(sess.run("weights:0"))
    graph = tf.get_default_graph()


    Qout = graph.get_tensor_by_name("Qout:0")
    #l1 = tf.matmul(inputs1,weights["w1"])
    #l2 = tf.matmul(l1,weights["w2"])
    #l3 = tf.matmul(l2,weights["w3"])
    #Qout = tf.matmul(l2,weights["wout"])
    #W = tf.Variable(tf.random_uniform([16,4],0,0.01))
    #Qout= tf.matmul(inputs1,W)
    predict = graph.get_tensor_by_name("predict:0")




    inputs1 = graph.get_tensor_by_name("input:0")
    nextQ = graph.get_tensor_by_name("nextQ:0")
    loss =  graph.get_tensor_by_name("loss:0")
    #trainer = graph.get_tensor_by_name("trainer:0")
    trainer = tf.train.AdamOptimizer(learning_rate=0.1)
    updateModel = trainer.minimize(loss)
    init = tf.global_variables_initializer()

    sess.run(init)
    print(sess.run("weights:0"))
    saver.restore(sess, tf.train.latest_checkpoint('./tmp'))
    print(sess.run("weights:0"))
    #some_ini = tf.variables_initializer([nextQ])
    #updateModel = trainer.minimize(loss, name="updateModel")
    #updateModel = graph.get_tensor_by_name("updateModel:0")

    #init = tf.initialize_all_variables()

    # Set learning parameters
    y = .99
    e = 0.1
    num_episodes = 20000
    #create lists to contain total rewards and steps per episode
    jList = []
    rList = []

    #saver = tf.train.Saver()

    #Run the operation by feeding input

    #Prints 24 which is sum of (w1+w2)*b1




    #sess.run(init)
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
            updated_model = sess.run([updateModel],feed_dict={inputs1:np.identity(num_input)[s:s+1],nextQ:targetQ})
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
    #This will print 60 which is calculated