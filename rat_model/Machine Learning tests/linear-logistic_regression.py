#!/usr/bin/env python
# Multiple Linear/Logistic Regression tests

# Importing the libraries
import numpy as np
import matplotlib.pyplot as plt
import pandas as pd

# Importing the dataset
dataset = pd.read_csv('pd_stimu_fulldata.csv')
X = dataset.iloc[:, :3].values
y = dataset.iloc[:, 3].values
dataset_validation = pd.read_csv('pd_stimu.csv')
X_val = dataset_validation.iloc[:, [3,4,5]].values
y_val= dataset_validation.iloc[:, 0].values

# Splitting the dataset into the Training set and Test set
from sklearn.model_selection import train_test_split
X_train, X_test, y_train, y_test = train_test_split(X, y, test_size = 0.2, random_state = 0)


# Feature Scaling
from sklearn.preprocessing import StandardScaler, MinMaxScaler
sc = StandardScaler()
#min_max_scaler = MinMaxScaler()
X_train = sc.fit_transform(X_train)
X_test = sc.transform(X_test)

# Training the Multiple Linear Regression model on the Training set
from sklearn.linear_model import LinearRegression, LogisticRegression
regressor = LinearRegression()
#regressor = LogisticRegression()
regressor.fit(X_train, y_train)

# Predicting the Test set results
y_pred = regressor.predict(X_test)

#np.set_printoptions(precision=2)
#print(np.concatenate((y_pred.reshape(len(y_pred),1), y_test.reshape(len(y_test),1)),1))

#result=np.array([[9.125,116.0669,19.4791]]) #01
#result=np.array([[9.375,108.4864,16.5823]]) #01
#result=np.array([[9.625,106.5393,7.0927]]) #11 
#result=np.array([[4,233.1533,15.474]])#10
#result=np.array([[4.75,225.91,46.5995]])#10
#result=np.array([[2.5,190.7,6.57]])#00
#result=np.array([[3.375,341.835,43.668]])#00
#result=np.array([[2.75,	226.265,9.485]])#00
y_pred1 = regressor.predict(sc.transform(X_val))


sum_0=[]
sum_1=[]
"""
for z in range(0,len(y_val)):
    if y_val[z]==0:
        sum_0.append(y_pred1[z])
    else:
        sum_1.append(y_pred1[z])
"""
print(y_pred1)
np.set_printoptions(precision=2)
print(np.concatenate((y_pred1.reshape(len(y_pred1),1), y_val.reshape(len(y_val),1)),1))
"""
mean0=np.mean(sum_0)
mean1=np.mean(sum_1)
std0=np.std(sum_0)
std1=np.std(sum_1)
"""
from sklearn.metrics import confusion_matrix, accuracy_score
#cm = confusion_matrix(y_test, y_pred)
#print(" accuracy")
#print(cm)
#print(accuracy_score(y_test, y_pred))
"""
# Visualising the Training set results
plt.scatter(X_train[:,0], y_train, color = 'red')
plt.plot(X_train[:,0], regressor.predict(X_train), color = 'blue')
plt.title('Fr and PD (Training set)')
plt.xlabel('Fr (Hz)')
plt.ylabel('PD or not ')
plt.show()
"""
"""
# Visualising the Test set results
plt.scatter(X_test[:,0], y_test, color = 'red')
plt.scatter(X_test[:,0], regressor.predict(X_test), color = 'blue')
#plt.plot(X_train[:,0], regressor.predict(X_train), color = 'blue')
plt.title('Salary vs Experience (Test set)')
plt.xlabel('Fr (Hz)')
plt.ylabel('Pd or not')
plt.show()
"""
# Evaluating the Model Performance
from sklearn.metrics import r2_score#, label_ranking_average_precision_score
print(r2_score(y_test, y_pred))