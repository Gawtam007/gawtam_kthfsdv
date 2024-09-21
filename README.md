# kthfsdv

## Exercise 1

- file structute of the packages
  ```
  ex1/
  │ ├── pub_package/
  │ ├── src/
  │ │ └── publisher.py
  │ ├── CMakeLists.txt
  │ └── package.xml
  ├── package_2/
  │ ├── src/
  │ │ └── subscriber.py
  │ ├── CMakeLists.txt
  │ └── package.xml
  ```

- Steps to run
  - add packages in the ros workspace and build it
    ```
    catkin_make
    ```
  - source the setup file
    ```
    source devel/setup.bash
    ```
  - start the roscore
    ```
    roscore
    ```
  - run publisher
    ```
    rosrun pub_package publisher.py
    ```
  - run subscriber
    ```
    rosrun sub_package subscriber.py
    ```


## Exercise 2

- Install dependencies
  ```
  pip install -r requirements.txt
  ```
- Run the python file


### Converting to Latex

Note: You might have to modify the colors.py in the webcolors package that is used by tikzplotlib (adding reference to the stackoverflow page that resolved the error in my case)
https://stackoverflow.com/questions/78672058/tikzplotlib-module-throws-attribute-error-module-webcolors-has-no-attribute

- To view the .tex file generated in Overleaf/any other latex editor

  - necessary latex packages
  ```
  \usepackage{tikz}
  \usepackage{pgfplots}
  \usepackage{pgfplotstable}
  \pgfplotsset{compat=1.7}
  \usepackage{subcaption}
  \usepgfplotslibrary{groupplots}
  ```
  - To add the tex file as image
  ```
  \begin{figure}
      \centering
      \input{kthfsplot}
      \caption{Matplotlib to Tikz}
      \label{fig:my_tikz}
  \end{figure}
  ```
