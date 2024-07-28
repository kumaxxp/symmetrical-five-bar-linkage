# Symmetrical Five-Bar Linkage Diagram

## PlantUML Code

Below is the PlantUML code used to generate the symmetrical five-bar linkage diagram.

``` plantuml
@startuml
!define RECTANGLE class
RECTANGLE B1
RECTANGLE M1
RECTANGLE X
RECTANGLE M2
RECTANGLE B2

B1 -[hidden]-> M1
M1 -[hidden]-> X
X -[hidden]-> M2
M2 -[hidden]-> B2
B2 -[hidden]-> B1

B1 - M1 : Link 1
M1 - X : Link 2
X - M2 : Link 3
M2 - B2 : Link 4
B2 - B1 : Link 5
@enduml

```

