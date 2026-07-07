2026-07-07

# Project Subfolders
## exercises:
- preliminary trajectory optimization problems including the 'dragracer' for direct collocation and a 1-dimensional mass-spring model
- not maintained for readability and clarity. mainly just to have
## model:
- contains the working models for the project analyses
-   2D SLIP model with leg inertia.
-   main scripts (trajOptim, paramAnalysis)
-   functions:
    - **single optimization**: developed first...to run a single optimization. useful for debugging and single trial visualization, but not automatic (used for trajOptim.m)
    - **multi optimization**: developed based on single optimization trials. able to optimize COT automatically. used in parameter analyses (paramAnalysis.m)
    - **shared functions**: functions common to both single and multi optimziation
-   warmStartTemplates: generic optimal solutions to serve as a starting guess for optimization.
