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
    ``` catkin_make ```
  - source the setup file
    ``` catkin_make ```
  - start the roscore
    ```roscore ```
  - run publisher
    ``` rosrun pub_package publisher.py ```
  - run subscriber
    ``` rosrun sub_package subscriber.py ```


## Exercise 2


### Converting to Latex

https://stackoverflow.com/questions/78672058/tikzplotlib-module-throws-attribute-error-module-webcolors-has-no-attribute

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
