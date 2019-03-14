================================
Data probing using Probe module
================================

Pipeline is a living organism made of several pieces called modules, each
contributing in overall processing of audio data. When encountering distortions,
is it paramount to bisect pipeline in order to isolate segments which may or may
not be responsible for said issues. This idea has been embodied in data probing
and requires enlisting Probe module.

Probing allows for direct data extraction from or injection to target module,
providing opportunity to verify if processing done by the module is correct.
Note: parsing of extracted data is not part of this document and is considered
Intel internal only.

Probe is a loadable, standalone module i.e. there is no parent pipeline
assigned. By being assigned to no pipeline, it must be explicitly deleted by
sending Delete Instance IPC request once module is no longer needed. No more
than one Probe module may be initialized. Probe module by itself serves only as
a mediator - dockyard for dispatching all probing related IPC request.

**Requirements**

* DEBUG_FS enabled
* tinycompress. Please see tinycompress install and readme for its
  installation and setup of crecord and cplay tools


Instance initialization
-----------------------

UUID: 7CAD0808-AB10-CD23-EF45-12AB34CD56EF

Set ppl_instance_id to INVALID_PIPELINE_ID (-1) in Init Instance IPC request.
There is no dedicated execution context, thus core_id and proc_domain should be
set to 0. Module uses no cycles and input/ output frame sizes are unused.

struct skl_probe_mod_cfg describes the module configuration. Apart from base
configuration, it contains struct skl_probe_gtw_cfg field, which specifies
node_id and dma_buffer_size for extraction gateway.
Driver may choose to skip setting extraction gateway configuration by assigning
INVALID_NODE_ID (-1) instead. However, extraction will not be supported.


Connection purposes
-------------------
::

                        **************
                        *            * -----Out0----->
        ------In0-----> *            *
                        *   Module   * -----Out1----->
        ------In1-----> *            *
                        *            * -----Out2----->
                        **************             ^ each queue is a possible
                                                      probe connection point

Each module within pipeline can be described by its UUID, module_id and
instance_id attributes. Modules expose a number of pins for in and out
connections with other modules, effectively creating a path.
``InX(s)`` and ``OutY(s)`` denote pins (or queues as they are also called).

SKL_CONNECTION_PURPOSE_EXTRACT
    Extract data from module, given the module_id, instance_id and queue.

SKL_CONNECTION_PURPOSE_INJECT
    Inject data to module, given the module_id, instance_id and queue.

SKL_CONNECTION_PURPOSE_INJECT_REEXTRACT
    Sometimes we want to do both - extract from and inject data to the exact
    same queue for given module. This is not possible with previous two, thus
    INJECT_REEXTRACT has been designed as a solution to this limitation. It
    combines the two, starting with inject operation which is then followed by
    data extraction.


Limitations
-----------

There can be at most one extraction stream associated with Probe module, which
is done during its initialization and cannot be modified during the entire
lifetime of module.

Maximum number of probe points connected for extraction may vary between
platforms and firmware versions but should be no less than 10. In case of
injectors, number of active probe points is limited by count of available host
output streams. Currently, for most Intel platforms this number equals 9.
Maximum of one probe points connected per one injector stream.


IPC interface
-------------

Implementation offers six IPC requests:

LARGE_CONFIG_GET

- INJECTION_PROBE_DMA
    Retrieve list of host output DMAs associated with Probe module
- PROBE_POINTS
    Obtain the list of currently active probe points

LARGE_CONFIG_SET

- INJECTION_PROBE_DMA
    Associate host output DMA with Probe module for data injection
- INJECTION_PROBE_DMA_DETACH
    Detach one or more host DMAs from Probe module
- PROBE_POINTS
    Create one or more probe points
- PROBE_POINTS_DISCONNECT
    Disconnect one or more probe points


Compress adaptation
-------------------

open:

1. Assign required resources, that is streams, given the compress direction for
   later use

We do not want to occupy any other resources at this point until probing is
confirmed by set_params. No Probe module is initialized because no probe points
can be connected.

free:

1. Retrieve and disconnect all currently connected probe points for given DMA
2. If stream is of PLAYBACK direction, also retrieve and detach host output DMA
   assigned to this stream
3. Cleanup and free stream resources
4. If stream direction is CAPTURE (max one) ensure extractor is invalided before
   leaving
5. If no probing streams are left, send Delete Instance IPC request

set_params:

1. Allocate required stream resources
2. Calculate and set stream format
3. If it is the very first probing stream, send Init Instance IPC request for
   Probe module
4. If stream direction equals PLAYBACK, associate this stream DMA with Probe
   module

For the rest, source is pretty self explanatory.

Key thing to note is the probe operation ordering, which goes as follows:

1. Init Probe module
2. Attach host DMA to Probe module if direction of type PLAYBACK
3. Connect probe points
4. Disconnect probe points
5. Detach host DMA from Probe module if direction of type PLAYBACK
6. Delete probe module

One cannot proceed with connection of injection probe points until given
stream's DMA is associated with Probe module. Consequently, before detaching
injector DMA, all probe points for that stream should be disconnected.


User space
----------

Skylake driver exposes three debugfs entries designed to support four out of six
available IPC requests. These are:

General input format:
    u32,u32,u32,(...)

probe_injection_dma:

* dump list of assigned host output DMAs for injection

::

    cat probe_injection_dma

probe_points:

* connect new probe points
* dump currently connected probe points

::

    echo 0x10000004,0,0 > probe_points
    cat probe_points

struct skl_probe_point_desc consists of 3 u32 fields, having size of 12 bytes
total. To connect probe point using debugfs, simply write to probe_points
sequence of 3 u32s separated with ','. You can, however, enter numerous trios
causing several probe points connection in the process.

probe_points_disconnect:

* disconnect existing probe points

::

    echo 0x10000004 > probe_points_disconnect

INJECTION_PROBE_DMA (SET) and INJECTION_PROBE_DMA_DETACH (SET) are unsupported
as they are directly tied to strict initialization process of injection probe
where host stream becomes paired with said probe. Under no circumstances can
user interfere with these settings.

Using debugfs alone will not yield the expected result. It only allows to
configure the probes yet no processing is executed. In order to actually start
extracting or injecting data, compress stream must be started. Skylake provides
separate compress entries for extraction (CAPTURE) and injection (PLAYBACK).

Despite having no purpose without a separate PCM stream to validate modules for,
probe compress implementation allows for opening, running and disposing of
compress streams freely. Direction of stream of interest should be opposite to
probing direction, that is, data extraction (CAPTURE) targets running playback
stream and vice versa.

The most common use case scenario is data extraction:

1. Start the crecord to initialize and prepare for extraction
2. Start playback PCM stream using aplay tool
3. Pause PCM stream
4. Use probe debugfs ``probe_points`` entry found in ``/dsp/ipc/`` to connect
   probe points to target module within pipeline
5. Unpause PCM stream
6. Once finished, simply close aplay
7. Close crecord last, so no data from PCM stream is lost
8. Parse data from output file

Things get more complicated in case of INJECT_REEXTRACT purpose - we need the
power of cplay and crecord combined:

1. Start the crecord to initialize and prepare for extraction
2. Start cplay to associate host output DMA with Probe module
3. Start capture PCM stream using arecord tool
4. Pause PCM capture stream
5. Start playback PCM stream using aplay tool
6. Pause PCM playback stream
7. Use probe debugfs ``probe_points`` entry found in ``/dsp/ipc/`` to connect
   probe points to target module when connecting, specify same queue for
   extraction and injection
8. Unpause playback PCM stream
9. Unpause capture PCM stream
10. Once finished, simply close aplay and then arecord
11. Close cplay and then crecord, ensuring no data from PCM streams is lost
12. Parse data from output file

Note: DMA attach, detach, as well as probe points connections and disconnection
can be done either in bulk or one-by-one. Firmware offers no fallback mechanism
in failure scenario, thus if it happens to be, modify your test to sent requests
using one-by-one method rather than bulk. It will be easier to navigate which
probe point exactly is involved in the failure.
