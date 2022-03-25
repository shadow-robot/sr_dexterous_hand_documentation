class TestClass():
    def __init__(self):
        pass

    def add_two_integer_numbers(self, num1, num2):
        """
        [Method used to add two numbers.]
        ### Parameters
        1. num1 : int
            - An integer value
        2. num2 : int
            - An integer value
        ### Returns
        - int
            - A numeric value being the sum of the two arguments
        Raises
        ------
        - TypeError
            - Raised if at least one of the arguments is not integer
        ------
        This method does the same as :meth:`test_class.Test`
        """
        out = 0
        try:
            if not isinstance(num1, int) or not isinstance(num2, int):
                raise TypeError("One of the arguments is not an integer! Defaulting output to 0.")
            out = num1+num2
        except TypeError as e_1:
            print(e_1)
        return out
        
