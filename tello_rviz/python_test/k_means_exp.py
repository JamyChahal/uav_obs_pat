from sklearn.cluster import KMeans
import numpy as np

def main():
    X = np.array([[1, 2], [1, 4], [1, 0], [10, 2]])
    kmeans = KMeans(n_clusters=2, random_state=0).fit(X)
    print(kmeans.labels_)
    print(kmeans.predict(X))

if __name__ == '__main__':
    main()
