
###Por defecto
# import pandas
# import serial,time,csv
# from sklearn import model_selection
# from sklearn.linear_model import LogisticRegression
# import pickle

# names = ['tVal2, tVal3, tVal4, tVal5, tVal6, tVal7, ax, ay, az, gx, gy, gz, face']
# dataframe = pd.read_csv(url, names=names)
# array = dataframe.values
# X = array[:,0:8]
# Y = array[:,8]
# test_size = 0.33
# seed = 7
# X_train, X_test, Y_train, Y_test = model_selection.train_test_split(X, Y, test_size=test_size, random_state=seed)


# # Fit the model on training set
# model = LogisticRegression()
# model.fit(X_train, Y_train)
# # save the model to disk
# filename = 'DataFace.sav'
# pickle.dump(model, open(filename, 'wb'))
#  
# # some time later...
#  
# # load the model from disk
# loaded_model = pickle.load(open(/DataFace.sav 'rb'))
# result = loaded_model.score(X_test, Y_test)
# print(result)predicted=model.predict([data])


# model=pickle.load(open(filename, 'rb'))
# while (True):
#    data= serial.Serial('/dev/ttyACM0',9600)
# predicted=model.predict([data])
# pint("La prediccion es: " predicted)


##otro codigo como el primero

import serial
import pickle

# Cargar el modelo desde el archivo guardado
loaded_model = pickle.load(open('DataFace.sav', 'rb'))

# Abrir la conexión serial con el puerto /dev/ttyACM0 a 9600 baudios
ser = serial.Serial('/dev/ttyACM0', 9600)

while True:
    try:
        # Leer una línea de datos desde el puerto serial
        line = ser.readline().decode('ascii', errors='replace').rstrip().split(',')
        
        if len(line) >= 13:
            # Convertir los valores a números flotantes
            data = [float(x) for x in line[:12]]
            
            # Realizar la predicción usando el modelo cargado
            predicted = loaded_model.predict([data])
            
            # Mostrar la predicción en la consola
            print("La predicción es:", predicted)
    except KeyboardInterrupt:
        print("Exiting")
        break

# Cerrar la conexión serial
ser.close()






###Para no entrenado
# import serial
# import pickle

# # Cargar el modelo desde el archivo guardado
# loaded_model = pickle.load(open('DataFace.sav', 'rb'))

# # Abrir la conexión serial con el puerto /dev/ttyACM0 a 9600 baudios
# ser = serial.Serial('/dev/ttyACM0', 9600)

# while True:
#     try:
#         # Leer una línea de datos desde el puerto serial
#         line = ser.readline().decode('ascii', errors='replace').rstrip().split(',')
        
#         if len(line) >= 13:
#             # Convertir los valores a números flotantes
#             data = [float(x) for x in line[:12]]
            
#             # Realizar la predicción usando el modelo cargado
#             predicted = loaded_model.predict([data])
            
#             # Mostrar la predicción en la consola
#             print("La predicción es:", predicted)
#     except KeyboardInterrupt:
#         print("Exiting")
#         break

# # Cerrar la conexión serial
# ser.close()

