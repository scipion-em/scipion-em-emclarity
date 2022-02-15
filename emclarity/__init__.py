# **************************************************************************
# *
# * Authors:     you (you@yourinstitution.email)
# *
# * your institution
# *
# * This program is free software; you can redistribute it and/or modify
# * it under the terms of the GNU General Public License as published by
# * the Free Software Foundation; either version 2 of the License, or
# * (at your option) any later version.
# *
# * This program is distributed in the hope that it will be useful,
# * but WITHOUT ANY WARRANTY; without even the implied warranty of
# * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# * GNU General Public License for more details.
# *
# * You should have received a copy of the GNU General Public License
# * along with this program; if not, write to the Free Software
# * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA
# * 02111-1307  USA
# *
# *  All comments concerning this program package may be sent to the
# *  e-mail address 'scipion@cnb.csic.es'
# *
# **************************************************************************

import os

import pwem
import pyworkflow as pw
from pyworkflow.utils import Environ
from .constants import *

__version__ = '0.0.1'

_logo = "icon.png"
_references = ['you2019']


class Plugin(pwem.Plugin):
    _homeVar = EMCLARITY_HOME
    _pathVars = [EMCLARITY_HOME]

    @classmethod
    def _defineVariables(cls):
        emclarityHome = 'emClarity-%s' % __version__
        matlabMcrHome = 'matlabMcr-%s' % __version__
        cls._defineEmVar(EMCLARITY_HOME, emclarityHome)
        cls._defineEmVar(MATLAB_MCR_HOME, matlabMcrHome)

    @classmethod
    def getEnviron(cls):
        """ Setup the environment variables needed to launch EMCLARITY. """
        environ = Environ(os.environ)
        matlab_runtimeLib = os.path.join(MATLAB_FOLDER, 'v96/runtime/glnxa64')
        matlab_binLib = os.path.join(MATLAB_FOLDER, 'v96/bin/glnxa64')
        matlab_sysLib = os.path.join(MATLAB_FOLDER, 'v96/sys/os/glnxa64')
        matlab_externLib = os.path.join(MATLAB_FOLDER, 'v96/extern/bin/glnxa64')

        environ.update({
            'PATH': Plugin.getHome(),
            'EMCLARITY_BIN': os.path.join(Plugin.getHome(), 'bin'),
            'LD_LIBRARY_PATH': matlab_runtimeLib + ":" + matlab_binLib + ":" + matlab_sysLib
                                + ":" + matlab_externLib + ":" + environ['LD_LIBRARY_PATH']
        }, position=Environ.BEGIN)

        return environ

    @classmethod
    def getEMCLARITYEnvActivation(cls):
        activation = cls.getVar(EMCLARITY_ENV_ACTIVATION)
        scipionHome = pw.Config.SCIPION_HOME + os.path.sep

        return activation.replace(scipionHome, "", 1)

    @classmethod
    def isVersionActive(cls):
        return cls.getActiveVersion().startswith(__version__)

    @classmethod
    def getDependencies(cls):
        # try to get CONDA activation command
        condaActivationCmd = cls.getCondaActivationCmd()
        neededProgs = ['wget']
        if not condaActivationCmd:
            neededProgs.append('conda')

        return neededProgs

    @classmethod
    def runEMCLARITY(cls, protocol, program, args, cwd=None):
        """ Run EMCLARITY command from a given protocol. """
        protocol.runJob(program, args, env=cls.getEnviron(), cwd=cwd,
                        numberOfMpi=1)

    @classmethod
    def defineBinaries(cls, env):
        env.addPackage('emClarity',
                       version=__version__,
                       tar='emClarity_1.5.3.11.tgz',
                       default=True)

        destinationFolder = os.path.join(MATLAB_FOLDER,'matlab2019a')
        args = '-mode silent -agreeToLicense yes -destinationFolder %s' % os.path.abspath(destinationFolder)

        installCmd = 'cd %s && mkdir %s && ./install %s' % (MATLAB_FOLDER,
                                                            destinationFolder, args)

        matlab_cmd = [(installCmd, [])]

        env.addPackage('matlab_mcr2019a',
                       version=__version__,
                       commands=matlab_cmd,
                       tar='matlab_mcr2019a.tgz',
                       default=True)
