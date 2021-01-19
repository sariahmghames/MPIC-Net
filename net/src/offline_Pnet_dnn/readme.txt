We use a feed-forward deep neural network, parameterized by θ, to perform planning. Given
the obstacles encoding Z, current state x_t and the goal
state x_T , Pnet predicts the next state x̂_t+1 ∈ X_free which
would lead a robot closer to the goal region, i.e., x̂_t+1 =
Pnet((x_t , x_T , Z); θ)

train the model by running train.py

run tests using neuralplanner.py
