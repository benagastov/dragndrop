import streamlit as st

# Define the questions and answers
questions = [
    {
        "question": "What is the capital of France?",
        "choices": ["Paris", "London", "New York", "Tokyo"],
        "answer": "Paris"
    },
    {
        "question": "What is the largest ocean in the world?",
        "choices": ["Atlantic Ocean", "Indian Ocean", "Arctic Ocean", "Pacific Ocean"],
        "answer": "Pacific Ocean"
    }
]

# Create a sidebar for the game
st.sidebar.title("Multiple Choice Game")

# Loop through the questions
for i, question in enumerate(questions):
    # Display the question
    st.header(question["question"])

    # Create radio buttons for the multiple choice answers
    choice = st.radio("Select an answer:", question["choices"])

    # Create a submit button
    if st.button("Submit", key=f"submit-{i}"):
        # Check the answer
        if choice == question["answer"]:
            st.write("Correct!")
        else:
            st.write("Incorrect. The correct answer is:", question["answer"])
