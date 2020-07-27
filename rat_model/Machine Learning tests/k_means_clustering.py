#! /usr/bin/env python

#The template is issued from Udemy course on machine learning //XXXXXXX

# K-Means Clustering

# Importing the libraries
import numpy as np
import matplotlib.pyplot as plt
import pandas as pd
from mpl_toolkits import mplot3d
# Importing the dataset
dataset = pd.read_csv('pd_stimu_fulldata.csv')
X = dataset.iloc[:, [3,0,1,2]].values
y = dataset.iloc[:, 3].values
#have to determine the first centroid else they will be randomly distributed
#init_centroid= dataset.iloc[:4, [5,4]].values
#print(init_centroid)
# Using the elbow method to find the optimal number of clusters
from sklearn.cluster import KMeans
print(len(y))
"""
wcss = []
for i in range(1, 11):
    kmeans = KMeans(n_clusters = i, init = 'k-means++', random_state = 42)
    kmeans.fit(X)
    wcss.append(kmeans.inertia_)
plt.plot(range(1, 11), wcss)
plt.title('The Elbow Method')
plt.xlabel('Number of clusters')
plt.ylabel('WCSS')
plt.show()
"""
# Training the K-Means model on the dataset
kmeans = KMeans(n_clusters = 4, init ='k-means++', random_state = 42)
y_kmeans = kmeans.fit_predict(X)
#print(X)
#a=np.array([[0,  3.15]])

#print(a)
#result_kmeans=kmeans.predict(a)
#print(int(result_kmeans))

# Creating figure 
fig = plt.figure(figsize = (10, 7)) 
ax = plt.axes(projection ="3d") 
# Visualising the clusters

ax.scatter3D(X[y_kmeans == 0, 0], X[y_kmeans == 0, 1],X[y_kmeans == 0, 2], s = 60, c = 'red', label = 'Healthy stimulated')
ax.scatter3D(X[y_kmeans == 1, 0], X[y_kmeans == 1, 1],X[y_kmeans == 1, 2], s = 60, c = 'blue', label = 'PD non stimulated')
ax.scatter3D(X[y_kmeans == 2, 0], X[y_kmeans == 2, 1],X[y_kmeans == 2, 2], s = 60, c = 'orange', label = 'Healthy non stimulated')
ax.scatter3D(X[y_kmeans == 3, 0], X[y_kmeans == 3, 1],X[y_kmeans == 3, 2], s = 60, c = 'cyan', label = 'PD stimulated')
ax.scatter3D(kmeans.cluster_centers_[:, 0], kmeans.cluster_centers_[:, 1],kmeans.cluster_centers_[:, 2], s = 50, c = 'yellow', label = 'Centroids')
plt.title('Clusters of ISI std/ISI mean')
ax.set_xlabel('Healthy/PD')
ax.set_ylabel('ISI std (ms)')
ax.set_zlabel('ISI means (ms)')
plt.legend()
plt.show()


"""
plt.scatter(X[y_kmeans == 0, 0], X[y_kmeans == 0, 1], s = 60, c = 'red', label="stimulated model")
plt.scatter(X[y_kmeans == 1, 0], X[y_kmeans == 1, 1], s = 60, c = 'red')
plt.scatter(X[y_kmeans == 2, 0], X[y_kmeans == 2, 1], s = 60, c = 'red')
plt.scatter(X[y_kmeans == 3, 0], X[y_kmeans == 3, 1], s = 60, c = 'cyan',label="non stimulated model")
#plt.scatter(kmeans.cluster_centers_[:, 0], kmeans.cluster_centers_[:, 1], s = 50, c = 'yellow', label = 'Centroids')
plt.title('Clusters based on 400 data')
plt.xlabel('Healthy / PD')
plt.ylabel('CTX_R Frequency rate (Hz)')

plt.legend()
plt.show()
"""