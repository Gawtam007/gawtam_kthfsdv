# kthfsdv

## Exercise 1


```
src/ ├── / │ ├── src/ │ │ ├── node1.py │ │ └── node2.py │ ├── CMakeLists.txt │ └── package.xml ├── package_2/ │ ├── src/ │ │ └── node3.cpp │ ├── CMakeLists.txt │ └── package.xml └── CMakeLists.txt
```


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
