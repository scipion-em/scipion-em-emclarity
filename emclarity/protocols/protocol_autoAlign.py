# -*- coding: utf-8 -*-
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
# *  e-mail address 'you@yourinstitution.email'
# *
# **************************************************************************


"""
Describe your python module here:
This module will provide the traditional Hello world example
"""
import os

from pyworkflow.protocol import Protocol

import imod.utils as utils
import numpy as np
import pwem.objects as data
from pwem.objects import Transform
from pyworkflow import BETA
from pyworkflow.object import Set, Integer
import pyworkflow.protocol.params as params
import tomo.objects as tomoObj
from imod import Plugin
from pwem.emlib.image import ImageHandler
from imod.protocols.protocol_base import ProtImodBase


class EmclarityAutoAlign(Protocol):
    """
    This protocol will print hello world in the console
    IMPORTANT: Classes names should be unique, better prefix them
    """
    _label = 'autoAlign'

    # -------------------------- DEFINE param functions ----------------------
    def _defineParams(self, form):
        """ Define the input parameters that will be used.
        Params:
            form: this is the form to be populated with sections and params.
        """
        # You need a params to belong to a section:
        form.addSection('Input')

        form.addParam('inputSetOfTiltSeries',
                      params.PointerParam,
                      pointerClass='SetOfTiltSeries',
                      important=True,
                      label='Input set of tilt-series.')

        form.addParam('beadDiameter', params.FloatParam,
                      default=10,
                      important=True,
                      label='Bead diameter',
                      help='Bead diameter in meters (e.g. 10e-9). If 0, the beads are ignored during the alignment')

        form.addParam('autoAli_max_resolution', params.FloatParam,
                      default=18,
                      label='Max Resolution',
                      help='Low-pass cutoff, in Å, used in alignment')

        form.addParam('autoAli_min_sampling_rate', params.FloatParam,
                      default=10,
                      label='Min sampling rate',
                      help='Maximum pixel size used for alignment, in Å per pixel')

        form.addParam('autoAli_max_sampling_rate', params.FloatParam,
                      default=3,
                      label='Max sampling rate',
                      help='Minimum pixel size used for alignment, in Å per pixel')

        form.addParam('autoAli_iterations_per_bin', params.FloatParam,
                      default=3,
                      label='Iterations per bin',
                      help='The number of patch tracking iterations, for each bin')

        form.addParam('autoAli_n_iters_no_rotation', params.FloatParam,
                      default=3,
                      label='n iters no rotation',
                      help='The number of patch tracking iterations, for each bin,'
                           'before activating local alignments')

        form.addParam('autoAli_patch_size_factor', params.FloatParam,
                      default=4,
                      label='Patch size factor',
                      help='Sets the size of the patches used for patch tracking.'
                           'Making this larger will result in more patches, and more local areas in later iterations,'
                           'but may also decrease accuracy')

        form.addParam('autoAli_patch_tracking_border', params.FloatParam,
                      default=64,
                      label='Patch tracking border',
                      help='Number of pixels to trim off each edge in X and in Y')

        form.addParam('autoAli_patch_tracking_border', params.FloatParam,
                      default=64,
                      label='Patch tracking border',
                      help='Number of pixels to trim off each edge in X and in Y')

        form.addParam('autoAli_patch_overlap', params.FloatParam,
                      default=0.5,
                      label='Patch overlap',
                      help='Fractional overlap in X and Y between patches that are tracked by correlation.'
                           'This influences the number of patches')

        form.addParam('autoAli_max_shift_in_angstroms', params.FloatParam,
                      default=40,
                      label='Max shift in angstroms',
                      help='Maximum shifts allowed, in Å, for the patch tracking alignment')

        form.addParam('autoAli_max_shift_factor', params.FloatParam,
                      default=1,
                      label='Max shift factor',
                      help='The maximum shifts allowed are progressively reduced with the iterations i')

        form.addParam('autoAli_refine_on_beads', params.BooleanParam,
                      default=False,
                      label='Refine on beads',
                      help='Whether or not the patch tracking alignment should be refined using the gold beads.'
                           'This refinement makes the alignment significantly slower, but can substantially '
                           'improve the quality of the alignment')

    # --------------------------- STEPS functions ------------------------------
    def _insertAllSteps(self):
        # Insert processing steps
        self._insertFunctionStep(self.autoAlignStep)
        self._insertFunctionStep(self.createOutputStep)

        for ts in self.inputSetOfTiltSeries.get():
        #     self._insertFunctionStep(self.convertInputStep, ts.getObjId())
            self._insertFunctionStep(self.autoAlignStep, ts.getObjId())
        #     self._insertFunctionStep(self.generateOutputStackStep, ts.getObjId())
        #     if self.computeAlignment.get() == 0:
        #         self._insertFunctionStep(self.computeInterpolatedStackStep, ts.getObjId())
        # self._insertFunctionStep(self.closeOutputSetsStep)

    def autoAlignStep(self):
        """Compute transformation matrix for each tilt series"""
        ts = self.inputSetOfTiltSeries.get()[tsObjId]
        tsId = ts.getTsId()
        extraPrefix = self._getExtraPath(tsId)
        tmpPrefix = self._getTmpPath(tsId)

        paramsXcorr = {
            'input': os.path.join(tmpPrefix, ts.getFirstItem().parseFileName()),
            'output': os.path.join(extraPrefix, ts.getFirstItem().parseFileName(extension=".prexf")),
            'tiltfile': os.path.join(tmpPrefix, ts.getFirstItem().parseFileName(extension=".tlt")),
            'rotationAngle': ts.getAcquisition().getTiltAxisAngle(),
            'filterSigma1': self.filterSigma1.get(),
            'filterSigma2': self.filterSigma2.get(),
            'filterRadius1': self.filterRadius1.get(),
            'filterRadius2': self.filterRadius2.get()
        }

        argsXcorr = "-input %(input)s " \
                    "-output %(output)s " \
                    "-tiltfile %(tiltfile)s " \
                    "-RotationAngle %(rotationAngle).2f " \
                    "-FilterSigma1 %(filterSigma1)f " \
                    "-FilterSigma2 %(filterSigma2)f " \
                    "-FilterRadius1 %(filterRadius1)f " \
                    "-FilterRadius2 %(filterRadius2)f "

        if self.cumulativeCorr == 0:
            argsXcorr += " -CumulativeCorrelation "

        Plugin.runImod(self, 'tiltxcorr', argsXcorr % paramsXcorr)

        paramsXftoxg = {
            'input': os.path.join(extraPrefix, ts.getFirstItem().parseFileName(extension=".prexf")),
            'goutput': os.path.join(extraPrefix, ts.getFirstItem().parseFileName(extension=".prexg")),
        }
        argsXftoxg = "-input %(input)s " \
                     "-goutput %(goutput)s"
        Plugin.runImod(self, 'xftoxg', argsXftoxg % paramsXftoxg)

    def generateOutputStackStep(self):
        # register how many times the message has been printed
        # Now count will be an accumulated value
        timesPrinted = Integer(self.times.get() + self.previousCount.get())
        self._defineOutputs(count=timesPrinted)

    # --------------------------- INFO functions -----------------------------------
    def _summary(self):
        """ Summarize what the protocol has done"""
        summary = []

        if self.isFinished():
            summary.append("This protocol has printed *%s* %i times." % (self.message, self.times))
        return summary

    def _methods(self):
        methods = []

        if self.isFinished():
            methods.append("%s has been printed in this run %i times." % (self.message, self.times))
            if self.previousCount.hasPointer():
                methods.append("Accumulated count from previous runs were %i."
                               " In total, %s messages has been printed."
                               % (self.previousCount, self.count))
        return methods
