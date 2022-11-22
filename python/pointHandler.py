import tensorflow as tf
import numpy as np
from tensorflow import keras
from keras import Sequential
from keras import layers
from keras.layers import Dense, Input, Flatten


def CarlaLossFunction(chosenPoint, carPoints, pointTLOs):
    if chosenPoint in carPoints:
        index = np.argwhere(chosenPoint == carPoints)[0][0]
        return 1-(float(pointTLOs[index])/(10000.0*1))  # loss function = 1 - (TLO/maxTLO*numOfSensors)
    else:
        return 0.0


def readInCarPoints():
    # import training data
    inputFile = open("/Users/jordan/Documents/Schoolwork/BachelorArbeit/DeepLProject/cmake-build-debug/trainingPoints3.pts", "r")
    numOfPoints = int(inputFile.readline().split(':')[1])

    inputPoints = np.empty(0, float)
    inputValues = np.empty(0, int)

    for i in range(numOfPoints):
        line = inputFile.readline()
        point = line.split('|')[0]
        inputPoints = np.append(inputPoints, float(point.split(',')[0]))
        inputPoints = np.append(inputPoints, float(point.split(',')[1]))
        inputPoints = np.append(inputPoints, float(point.split(',')[2]))
        inputValues = np.append(inputValues, float(line.split('|')[1]))

    # reshape arrays so they're easier to work with
    inputPoints = inputPoints.reshape(-1, 3)
    inputTensor = tf.convert_to_tensor(inputPoints)
    inputValues = inputValues.reshape(-1, 1)
    totalLidarTensor = tf.convert_to_tensor(inputValues)

    # train_dataset = tf.data.Dataset.from_tensor_slices((inputPoints, inputValues))

    inputs = keras.Input(shape=(3,), name="points")
    x1 = layers.Dense(64, activation="relu")(inputs)
    x2 = layers.Dense(64, activation="relu")(x1)
    outputs = layers.Dense(3, name="predictions")(x2)
    model = keras.Model(inputs=inputs, outputs=outputs)
    batch_size = 20

    # Instantiate an optimizer.
    optimizer = keras.optimizers.SGD(learning_rate=1e-3)
    # Instantiate a loss function.
    loss_fn = keras.losses.SparseCategoricalCrossentropy(from_logits=True)

    # Prepare the training dataset.
    train_dataset = tf.data.Dataset.from_tensor_slices((inputTensor, totalLidarTensor))
    train_dataset = train_dataset.shuffle(buffer_size=1024).batch(batch_size)

    # Prepare the validation dataset.
    # val_dataset = tf.data.Dataset.from_tensor_slices((x_val, y_val))
    # val_dataset = val_dataset.batch(batch_size)

    epochs = 2
    for epoch in range(epochs):
        print("\nStart of epoch %d" % (epoch,))

        # Iterate over the batches of the dataset.
        for step, (x_batch_train, y_batch_train) in enumerate(train_dataset):

            # Open a GradientTape to record the operations run
            # during the forward pass, which enables auto-differentiation.
            with tf.GradientTape() as tape:

                # Run the forward pass of the layer.
                # The operations that the layer applies
                # to its inputs are going to be recorded
                # on the GradientTape.
                logits = model(x_batch_train, training=True)  # Logits for this minibatch

                # Compute the loss value for this minibatch.
                loss_value = loss_fn(y_batch_train, logits)
                print(loss_value)

            # Use the gradient tape to automatically retrieve
            # the gradients of the trainable variables with respect to the loss.
            grads = tape.gradient(loss_value, model.trainable_weights)

            # Run one step of gradient descent by updating
            # the value of the variables to minimize the loss.
            optimizer.apply_gradients(zip(grads, model.trainable_weights))

            # Log every 200 batches.
            if step % 200 == 0:
                print(
                    "Training loss (for one batch) at step %d: %.4f"
                    % (step, float(loss_value))
                )
                print("Seen so far: %s samples" % ((step + 1) * batch_size))


