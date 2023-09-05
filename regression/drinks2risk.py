import pandas as pd
from sklearn.preprocessing import LabelEncoder, StandardScaler
from tensorflow.keras.models import Sequential
from tensorflow.keras.layers import Dense

# load the dataset
dataset = pd.read_csv('data/dataset.csv')

# encode the 'location', 'race', and 'gender' columns
le_location = LabelEncoder()
le_race = LabelEncoder()
le_gender = LabelEncoder()  # Added
dataset['location'] = le_location.fit_transform(dataset['location'])
dataset['race'] = le_race.fit_transform(dataset['race'])
dataset['gender'] = le_gender.fit_transform(dataset['gender'])  # Added

# split the dataset into input features (X) and output variable (y)
X = dataset.iloc[:, :-1].values
y = dataset.iloc[:, -1].values

# normalize the input features using StandardScaler
sc = StandardScaler()
X = sc.fit_transform(X)

# build the model
model = Sequential()
model.add(Dense(12, input_dim=6, activation='relu'))  # Updated input_dim to 6
model.add(Dense(8, activation='relu'))
model.add(Dense(1, activation='linear'))
model.compile(loss='mean_squared_error', optimizer='adam')

# train the model
model.fit(X, y, epochs=150, batch_size=10)

# make predictions on new data
new_data = [[le_location.transform(['Los Angeles'])[0], 0, 130, 65, le_race.transform(['White'])[0], le_gender.transform(['female'])[0]]]  # Added gender
new_data_scaled = sc.transform(new_data)
prediction = model.predict(new_data_scaled)
print(prediction)

