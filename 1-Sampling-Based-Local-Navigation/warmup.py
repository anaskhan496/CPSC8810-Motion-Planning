# warmup.py
# ---------
# Licensing Information:  You are free to use or extend these projects for
# educational purposes provided that (1) you do not distribute or publish
# solutions, (2) you retain this notice, and (3) you provide clear
# attribution to Clemson University and the authors.
# 
# Authors: Pei Xu (peix@g.clemson.edu) and Ioannis Karamouzas (ioannis@g.clemson.edu)
#


"""
  This code will allow you to verify the the successfull installation of 
  Python on your machine. If you want to practice a bit your Python skills, 
  please modify the functions below as indicated. 
"""

def max_index(data):
    """
    Find the index of the maximum value in the given list.

    Parameters
    ----------
    data: List[float],
          a list of floats

    Returns
    -------
    index: int,
           the index of any of the maximum values in the given data, i.e. 
           data[index] == max(data)
    """
    index = None
    ###########################################################################
    # Please finish the code here
    # Can you implement the function with just one line of code?

    ###########################################################################
    return index


def two_sum(nums, target):
    """
    Return two values in nums such that their sum is equal to the target value

    Parameters
    ----------
    nums: List[int],
          a list of integers
    target: int,
            the target value
    
    Returns
    -------
    value1: int
    value2: int
        value1 and value2 are two numbers in nums and their sum is equal to target
    """
    value1, value2 = None, None
    ###########################################################################
    # Please finish the code here

    ###########################################################################
    return value1, value2


def factorial(n):
    """
    Compute the factorial of the number n using recursion.

    Factorial of a number n is defined as:
    n! = 1, if n = 0 or n = 1 
    n! = 1 * 2 * ... * (n-1) * n = n * (n-1)!, if n > 1

    Recursion is a way to solve a problem by calling the function itself to solve
    subproblems of that problem. Please try to compute the factorial in a recursive
    manner.

    Parameters
    ----------
    n: int, 
       a positive integer; its factorial is assumed in the valid range of python int data type.
    
    Returns
    -------
    the factorial of the given positive number
    """

    ###########################################################################
    # Please finish the code here
    pass
    ###########################################################################


if __name__ == "__main__":
    import sys
    print("Python {}".format(sys.version))
    assert(sys.version_info[0]==3 and sys.version_info[1]==6)
    import tkinter
    print("Tkinter {}".format(tkinter.TkVersion))
    import numpy as np
    print("Numpy {}".format(np.__version__))
    import matplotlib
    print("Matplotlib {}".format(matplotlib.__version__))
    
    c = input("\n"
        "Congrats! You have successfully installed Python 3.6.\n"
        "Please enter y to go on autograding or n to exit.\n"
        "[y/n] "
    )
    if c.lower() != "y": exit()
    
    data = list(np.random.rand(26))
    assert(data[max_index(data)] == max(data))

    nums = np.random.rand(26)
    tar = sum(nums[np.random.randint(0, 26, 2)])
    v1, v2 = two_sum(list(nums), tar)
    assert(v1 in nums and v2 in nums and v1+v2==tar)

    n = np.ranodm.rand(3, 10)
    lookup = [
        1, 1, 2, 6, 24,
        120, 720, 5040, 40320, 362880
    ]
    assert(lookup[n] == factorial(n))
