#!/usr/bin/env python
# Support Vector Regression (SVR)

# Importing the libraries
import numpy as np
import matplotlib.pyplot as plt
import pandas as pd

# Importing the dataset
dataset = pd.read_csv('pd_stimu_fulldata.csv')
X = dataset.iloc[:, :3].values
y = dataset.iloc[:, 3].values

y = y.reshape(len(y),1)
#print(y)

# Feature Scaling
from sklearn.preprocessing import StandardScaler
sc_X = StandardScaler()



# Splitting the dataset into the Training set and Test set
from sklearn.model_selection import train_test_split
X_train, X_test, y_train, y_test = train_test_split(X, y, test_size = 0.2, random_state = 0)


X_train = sc_X.fit_transform(X_train)
X_test = sc_X.transform(X_test)

# Training the SVR model on the training dataset
from sklearn.svm import SVR
regressor = SVR(kernel = 'rbf')
regressor.fit(X_train, y_train)

y_pred=regressor.predict(X_test)
# Predicting a new result
#y_test.inverse_transform(regressor.predict())
print(len(X_train[:,0]))
print(len(y_train))
"""
# Visualising the SVR results
plt.scatter(sc_X.inverse_transform(X_train[:,0]), y_train, color = 'red')
plt.plot(sc_X.inverse_transform(X_train[:,0]), regressor.predict(X_train), color = 'blue')
plt.title('Truth or Bluff (SVR)')
plt.xlabel('Position level')
plt.ylabel('Salary')
plt.show()

# Visualising the SVR results (for higher resolution and smoother curve)
X_grid = np.arange(min(sc_X.inverse_transform(X)), max(sc_X.inverse_transform(X)), 0.1)
X_grid = X_grid.reshape((len(X_grid), 1))
plt.scatter(sc_X.inverse_transform(X), sc_y.inverse_transform(y), color = 'red')
plt.plot(X_grid, sc_y.inverse_transform(regressor.predict(sc_X.transform(X_grid))), color = 'blue')
plt.title('Truth or Bluff (SVR)')
plt.xlabel('Position level')
plt.ylabel('Salary')
plt.show()
"""

# Evaluating the Model Performance
from sklearn.metrics import r2_score#, label_ranking_average_precision_score
print(r2_score(y_test, y_pred))

print(" accuracy")
"""
print(accuracy_score(y_train,classifier.predict(X_train)))
print(accuracy_score(y_test, y_pred))
"""