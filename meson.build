project('glm_vs_eigen','cpp','c')

sources = ['glm-vs-eigen.cc','trackball.cc']

eigendep = dependency('eigen3')
glmdep = dependency('glm')
fmtdep = dependency('fmt')
deps = [eigendep,glmdep,fmtdep]

exe = executable('glm-vs-eigen',
                 sources,
                 dependencies: deps)
