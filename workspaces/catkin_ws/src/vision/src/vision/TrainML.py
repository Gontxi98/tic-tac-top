import matplotlib.pyplot as plt
import pandas as pd
import numpy as np
import statsmodels.api as sm
from sklearn.model_selection import train_test_split
from sklearn.metrics import mean_squared_error
from sklearn.metrics import r2_score,classification_report,confusion_matrix
from sklearn.tree import DecisionTreeClassifier, export_text
from sklearn import tree
from sklearn.neural_network import MLPClassifier

class GesturePredictor():
     def __init__(self) -> None:
          data = pd.read_csv("./src/vision/src/vision/Dataset/unified.csv",index_col=0)
          X = data.drop("Gesto",axis="columns").values
          Y = data["Gesto"].values
          X_train,X_test,Y_train,Y_test = train_test_split(X,Y,test_size=0.2,random_state=42)
          #clf = tree.DecisionTreeClassifier()
          #clf = clf.fit(X_train,Y_train)
          model = MLPClassifier(hidden_layer_sizes=(64,32),max_iter=100,random_state=42)
          model.fit(X_train,Y_train)
          self.classifier = model

     def predict_gesture(self,HandLandmark):
          y_pred = self.classifier.predict(HandLandmark)
          return y_pred