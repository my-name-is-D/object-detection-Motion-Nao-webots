#!/usr/bin/env python
# Artificial Neural Network
#https://scikit-learn.org/stable/modules/generated/sklearn.neural_network.MLPClassifier.html
# Importing the libraries
import numpy as np
import matplotlib.pyplot as plt
import pandas as pd
import tensorflow as tf
import csv
from sklearn.svm import SVC
from sklearn.datasets import load_digits
from sklearn.model_selection import learning_curve
from sklearn.model_selection import ShuffleSplit
import pickle
tf.__version__


def main():
    # Part 1 - Data Preprocessing

    # Importing the dataset
    dataset = pd.read_csv('pd_stimu_fulldata.csv')
    dataset_validation = pd.read_csv('pd_stimu.csv')
    X = dataset.iloc[:, :3].values
    #y = dataset.iloc[:, 4].values #if only PD
    y = dataset.iloc[:, [3,4]].values #if PD + stimu

    X_val = dataset_validation.iloc[:, [3,4,5]].values
    y_val= dataset_validation.iloc[:, [0,2]].values
    #print(len(X))
    #print(X)
    #print(y)

    
    # Splitting the dataset into the Training set and Test set
    from sklearn.model_selection import train_test_split
    X_train, X_test, y_train, y_test = train_test_split(X, y, test_size = 0.2, random_state = 0)

    # Feature Scaling
    from sklearn.preprocessing import StandardScaler
    #Multi-layer Perceptron is sensitive to feature scaling
    sc = StandardScaler()
    X_train = sc.fit_transform(X_train)
    X_test = sc.transform(X_test)
    #print(y_test)

    
    #TEST 1 : best result 70 with 4 data Rate (Hz)	ISI mean (ms)	ISI std  (ms)	mean std diff

    # Part 2 - Building the ANN

    # Initializing the ANN with Keras
    """

    from tensorflow.keras.wrappers.scikit_learn import KerasClassifier

    ann = tf.keras.models.Sequential()

    # Adding the input layer and the first hidden layer
    ann.add(tf.keras.layers.Dense(units=18,input_dim=3, kernel_initializer="he_normal",activation='relu'))

    # Adding the second hidden layer
    ann.add(tf.keras.layers.Dense(units=8, kernel_initializer="he_normal",activation='relu'))

    # Adding the third hidden layer
    ann.add(tf.keras.layers.Dense(units=7, kernel_initializer="he_normal", activation='relu'))

    # Adding the output layer
    ann.add(tf.keras.layers.Dense(units=2,  kernel_initializer="he_normal",activation='sigmoid'))
    # Compiling the ANN
    
    ann.compile(
        loss="categorical_crossentropy",
        optimizer='adam',
        metrics=[
            tf.keras.metrics.Accuracy(name="accuracy"),
            tf.keras.metrics.Precision(name="precision"),
            tf.keras.metrics.Recall(name="recall"),
            tf.keras.metrics.AUC(name="auc")])

    # Part 3 - Training the ANN
    clf = KerasClassifier(ann, epochs=500, batch_size=300)
    history=clf.fit(X_train, y_train, epochs=100, batch_size=1, verbose=0)
    clf.predict(X_test)

    #Ploting the learning curves
    plt.plot(history.history['accuracy'])
    plt.plot(history.history['val_accuracy'])
    plt.title('model accuracy')
    plt.ylabel('accuracy')
    plt.xlabel('epoch')
    plt.legend(['train', 'test'], loc='upper left')
    plt.show()

    """
   
    
    #TEST 2 : 0.875 -with 3HL 4N each (same data as test 1) | 0.8875 - with 4HL 4N each

    #TEST 3: 0.85 -with 4HL 6 5 8 8 (Rate (Hz)	ISI mean (ms)	ISI std  (ms))
    #        0.8625 -with 3HL 6,3, 6
    #0.85 6,5
    #0.8875 2HL 8,8   -- 0.9 3HL 9,9,9

    # Initializing the ANN with Sklearn
    from sklearn.neural_network import MLPClassifier
    alpha= 1e-5
    """
    #Code used to search for the best number of neuron in each layer
    filename = 'accuracy_HLN.csv'
    with open(filename, 'w') as csvFile:  
        # creating a csv writer object  
        field=['N1','N2','alpha','accuracy PD','accuracy train']
        writer=csv.DictWriter(csvFile,fieldnames=field)
        writer.writeheader()   
        for i in range (1,20):
            for n in range (1,20):
                for v in range (1,20):
    """
    
    #For small datasets, however, lbfgs can converge faster and perform better
    #activation is relu
    #lbfgs is an optimizer in the family of quasi-Newton methods.
    ann=  MLPClassifier(solver='lbfgs', alpha=alpha,
                        hidden_layer_sizes=(18,8,7), random_state=1,learning_rate='constant')
    #all the parameters of ann (output layer not comprised)
    param=ann.get_params()
    print(param)

    #SIMPLE FIT/PREDICT (to comment if we want to use "ACCURACY OVER EPOCHS")

    # Training the ANN on the Training set
    ann.fit(X_train, y_train)#, batch_size = 32, epochs = 100)#TEST 1

    # Predicting the Test set results
    y_pred = ann.predict(X_test)
    #y_pred = (y_pred > 0.5)

    """
    
    #ACCURACY OVER EPOCHS
    N_TRAIN_SAMPLES = X_train.shape[0]
    N_EPOCHS = 100
    N_BATCH = 128
    N_CLASSES = np.unique(y_train)

    scores_train = []
    scores_test = []

    # EPOCH
    epoch = 0
    while epoch < N_EPOCHS:
        print('epoch: ', epoch)
        # SHUFFLING
        random_perm = np.random.permutation(X_train.shape[0])
        mini_batch_index = 0
        while True:
            # MINI-BATCH
            indices = random_perm[mini_batch_index:mini_batch_index + N_BATCH]
            ann.partial_fit(X_train[indices], y_train[indices], classes=N_CLASSES)
            mini_batch_index += N_BATCH

            if mini_batch_index >= N_TRAIN_SAMPLES:
                break

        # SCORE TRAIN
        scores_train.append(ann.score(X_train, y_train))
        # SCORE TEST
        scores_test.append(ann.score(X_test, y_test))
        loss= ann.loss_curve_
        epoch += 1
    
    #Plot 
    plt.style.use('seaborn')
    fig, ax = plt.subplots(1, sharex=True, sharey=True)
    ax.plot(scores_train, c='red',label = 'Training learning curve')
    ax.plot(scores_test, c='blue',label = 'Testing learning curve')
    ax.legend(framealpha=1, frameon=False)
    ax.set_ylabel('Loss', fontsize = 14)
    ax.set_xlabel('Epoch', fontsize = 14)
    fig.suptitle("Accuracy over epochs", fontsize=14)
    plt.show()

    plt.style.use('seaborn')
    fig, ax = plt.subplots(1, sharex=True, sharey=True)
    ax.plot(loss, c='red',label = 'Model loss curve')
    ax.legend(framealpha=1, frameon=False)
    ax.set_ylabel('Loss', fontsize = 14)
    ax.set_xlabel('Epoch', fontsize = 14)
    fig.suptitle("Loss over epochs", fontsize=14)
    plt.show()
    """



    y_pred2 = ann.predict_proba(sc.transform(X_val))
    y_pred2_bin= ann.predict(sc.transform(X_val))
    sum_0=[]
    sum_1=[]
    for i in range(0,len(y_val)):
        if y_val[i][0]==0:
            sum_0.append(y_pred2[i][0])
        else:
            sum_1.append(y_pred2[i][0])

    #y_pred2 = (y_pred2 > 0.5)
    #print(y_pred2)
    #print(round(y_pred2[0][0],3))

    #17 wrong in 120 validation
    #33 wrong in 280 test
    #print(np.concatenate((y_pred.reshape(len(y_pred),1), y_test.reshape(len(y_test),1)),1)) #if only PD
    #print(np.concatenate((y_pred.reshape(len(y_pred),2), y_test.reshape(len(y_test),2)),1)) #if PD + stimu
    np.set_printoptions(precision=2)
    print(np.concatenate((y_pred2.reshape(len(y_pred2),2), y_val.reshape(len(y_val),2)),1))

    print("mean of the 0 proba and mean of the 1 proba", np.mean(sum_0), np.mean(sum_1))
    print("std of the 0 proba and mean of the 1 proba", np.std(sum_0), np.std(sum_1))
    # Making the Confusion Matrix
    from sklearn.metrics import  accuracy_score, classification_report,label_ranking_loss,  zero_one_loss,log_loss
    #cm =  multilabel_confusion_matrix(y_test, y_pred)
    #print(cm)
    accuracy=accuracy_score(y_test, y_pred)
    accuracy_train=accuracy_score(y_train, ann.predict(X_train))
    print("ranking loss",label_ranking_loss(y_val, y_pred2))
    print("zero one loss",zero_one_loss(y_test, y_pred, normalize=False))

    print("log_loss", log_loss(y_test, y_pred))
    print("average precision validation",accuracy_score(y_val,y_pred2_bin))
    #accuracy_check2=ann.score(y_test, y_pred)
    
    #print(i,n,v)
    print("average precision test",accuracy)
    #print(accuracy_check2)
    print("average precision train",accuracy_train)
    print(classification_report(y_train, ann.predict(X_train)))
    print(classification_report(y_test, y_pred))

    filename = '../ann'
    outfile = open(filename,'wb')


    pickle.dump(ann,outfile)
    outfile.close()

    filename = '../sc'
    outfile = open(filename,'wb')


    pickle.dump(sc,outfile)
    outfile.close()
    """
    #I am unsure what is really ploted there (i mean, what it represents) NOT USED

    train_sizes = np.linspace(0.1, 1.0, 15) #[1, 100, 400, 700,800,900,1000,1100,1200, 1300, 1400,1600,1750]
    train_sizes, train_scores, validation_scores = learning_curve(estimator = ann,X =X,
    y = y, train_sizes = train_sizes, cv = 8,
    scoring = 'accuracy')
    
    #WITH neg_mean_squared_error
    #train_scores_mean = -train_scores.mean(axis = 1)
    #validation_scores_mean = -validation_scores.mean(axis = 1)

    #WITH ACCURACY
    validation_scores_mean = np.mean(validation_scores, axis=1)
    train_scores_mean = np.mean(train_scores, axis=1)
    
    #print('Mean training scores\n\n', pd.Series(train_scores_mean, index = train_sizes))
    #print('\n', '-' * 20) # separator
    #print('\nMean validation scores\n\n',pd.Series(validation_scores_mean, index = train_sizes))

    
    plt.style.use('seaborn')
    plt.plot(train_sizes, train_scores_mean, label = 'Training learning curve')
    plt.plot(train_sizes, validation_scores_mean, label = 'Validation learning curve')
    plt.ylabel('accuracy', fontsize = 14)
    plt.xlabel('Training set size', fontsize = 14)
    plt.title('Learning curves for an ANN model', fontsize = 18, y = 1.03)
    plt.legend()
    #plt.ylim(0,2)
    plt.show()
    
    """
    # writing the fields               
                    #writer.writerow({'N1':i,'N2':n ,'alpha': alpha,
                    #'accuracy PD': accuracy,'accuracy train': accuracy_train})


if __name__ == '__main__':
    main()
        


            
 

