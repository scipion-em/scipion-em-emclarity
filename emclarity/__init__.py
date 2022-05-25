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
import imod as imodplugin

__version__ = '1.5.3.11'

_logo = "emClarity_Logo.png"
_references = ['himes2018']


class Plugin(pwem.Plugin):
    _homeVar = EMCLARITY_HOME
    _pathVars = [EMCLARITY_HOME]

    @classmethod
    def _defineVariables(cls):
        emclarityHome = 'emClarity_%s' % __version__
        matlabMcrHome = 'matlabMcr-%s' % __version__
        cls._defineEmVar(EMCLARITY_HOME, emclarityHome)
        cls._defineEmVar(MATLAB_MCR_HOME, matlabMcrHome)

    @classmethod
    def _getEMFolder(cls, version, *paths):
        return os.path.join("emClarity-%s" % version, *paths)

    @classmethod
    def _getEmClarityBin(cls, version, *paths):
        return os.path.join(cls._getEMFolder(version, "emClarity"), *paths)

    @classmethod
    def getEnviron(cls):
        """ Setup the environment variables needed to launch emClarity. """
        environ = Environ(os.environ)
        matlab_runtimeLib = os.path.join(MATLAB_FOLDER, 'matlab2019a/v96/runtime/glnxa64')
        matlab_binLib = os.path.join(MATLAB_FOLDER, 'matlab2019a/v96/bin/glnxa64')
        matlab_sysLib = os.path.join(MATLAB_FOLDER, 'matlab2019a/v96/sys/os/glnxa64')
        matlab_externLib = os.path.join(MATLAB_FOLDER, 'matlab2019a/v96/extern/bin/glnxa64')
        emclarity_bin = os.path.join(EMCLARITY_HOME, 'bin')
        emclarity_lib = os.path.join(EMCLARITY_HOME, 'lib')

        env_emC = {
            'LD_LIBRARY_PATH': (matlab_runtimeLib + ":" + matlab_binLib + ":" + matlab_sysLib + ":" + matlab_externLib + \
                                ":" + emclarity_bin + ":" + emclarity_lib + ":" + environ['LD_LIBRARY_PATH'])}

        environ.update(env_emC, position=Environ.BEGIN)

        return environ

    @classmethod
    def getEmClarityEnvActivation(cls):
        activation = cls.getVar(EMCLARITY_ENV_ACTIVATION)
        scipionHome = pw.Config.SCIPION_HOME + os.path.sep

        return activation.replace(scipionHome, "", 1)

    @classmethod
    def isVersionActive(cls):
        return cls.getActiveVersion().startswith(__version__)

    @classmethod
    def runEmClarity(cls, protocol, program, args, cwd=None):
        """ Run emClarity command from a given protocol. """
        # Get the command
        cmd = cls.getEmClarityCmd(program)

        protocol.runJob(cmd, args, env=cls.getEnviron(), cwd=cwd,
                        numberOfMpi=1)

    @classmethod
    def getEmClarityCmd(cls, program):
        """ Composes an EmClarity command for a given program. """

        cmd = ". " + imodplugin.getImodEnv()

        # Program to run
        program_path = cls._getProgram("emClarity_1_5_3_11_v19a")

        # Command to run
        cmd += program_path
        cmd += ' '
        cmd += program

        return cmd

    @classmethod
    def _getProgram(cls, program):
        """ Returns the same program  if config missing
        or the path to the program based on the config file."""
        # Compose path based on config
        progFromConfig = cls.getHome("bin", program)
        # Check if EmClarity from config exists
        if os.path.exists(progFromConfig):
            return progFromConfig
        else:
            return program

    @classmethod
    def defineBinaries(cls, env):
        env.addPackage('emClarity',
                       version=__version__,
                       tar='emClarity_1.5.3.11.tgz',
                       default=True)

        destinationFolder = os.path.join(MATLAB_FOLDER, 'matlab2019a')
        args = '-mode silent -agreeToLicense yes -destinationFolder %s' % os.path.abspath(destinationFolder)

        installCmd = 'cd %s && mkdir %s && ./install %s' % (MATLAB_FOLDER,
                                                            destinationFolder, args)

        matlab_cmd = [(installCmd, [])]

        env.addPackage('matlab_mcr2019a',
                       version=__version__,
                       commands=matlab_cmd,
                       tar='matlab_mcr2019a.tgz',
                       default=True)
