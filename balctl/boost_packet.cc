#include <stdlib.h>
#include <stdio.h>
#include <unistd.h>
#include <assert.h>
#include <math.h>

#include "libtlb/hacks.h"
#include "libtlb/loglevel.h"
#include "libtlb/tlbtemplates.h"

using namespace std;

#include <Python.h>
#include <boost/python/operators.hpp>
#include <boost/python/class.hpp>
#include <boost/python/module.hpp>
#include <boost/python/def.hpp>
#include <boost/python/tuple.hpp>
#include <boost/python/list.hpp>
#include <boost/python/str.hpp>
#include <boost/python/dict.hpp>
#include <boost/python/long.hpp>
#include <boost/python/extract.hpp>
#include <boost/python/numeric.hpp>
#include <boost/python/exception_translator.hpp>
#include <boost/python/reference_existing_object.hpp>
#include <boost/python/return_internal_reference.hpp>
#include <boost/python/manage_new_object.hpp>
#include <boost/python/return_value_policy.hpp>
#include <boost/python/copy_const_reference.hpp>
#include <boost/python/copy_non_const_reference.hpp>

#include "libatmel/packet_conn.h"
#include "libatmel/bootloader_conn.h"

#include "libtlb/tty_serial.h"

using namespace boost::python;

boost::python::object
packet_conn_rd_pkt(packet_conn *it)
{
  packet_rd *rd = it->rd_pkt();
  if (rd) {
    boost::python::str ret=boost::python::str(rd->buf, rd->n_buf);
    delete rd;
    return ret;
  } else {
    return boost::python::object();
  }
}

void
packet_conn_wr_pkt(packet_conn *it, boost::python::str data)
{
  packet_wr wr;
  
  int datalen;
  char *datap;
  if (PyString_AsStringAndSize(data.ptr(), &datap, &datalen)<0) abort();

  wr.add(datap, datalen);

  it->wr_pkt(wr);
}

void
bootloader_conn_upload_srec(bootloader_conn *it, const char *fn)
{
  it->upload_srec(fn);
}

static void debugentity_incr_verbose(const char *name) 
{
  debugentity::incr_verbose(string(name));
}
  


BOOST_PYTHON_MODULE(boost_packet)
{
  class_<tty_serial>("tty_serial", init<const char *>())
    .def("putc", &tty_serial::putc)
    //.def("puts", &abstract_serial::puts)
    .def("getc", &tty_serial::getc)
    .def("peekc", &tty_serial::peekc)
    .def("peekfor", &tty_serial::peekfor)
    .def("rx_empty", &tty_serial::rx_empty)
    .def("tx_drain", &tty_serial::tx_drain)
    .def("rx_discard", &tty_serial::rx_discard)
    .def("set_dtr", &tty_serial::set_dtr)
    .def("set_rts", &tty_serial::set_rts)
    .def("set_loopback", &tty_serial::set_loopback)
    .def("set_bitrate", &tty_serial::set_bitrate)
    .def("set_parity", &tty_serial::set_parity)
    .def("set_databits", &tty_serial::set_databits)
    .def("do_work", &tty_serial::do_work)
    ;

  class_<packet_conn>("packet_conn", init<tty_serial *>())
    .def("rd_pkt", &packet_conn_rd_pkt)
    .def("wr_pkt", &packet_conn_wr_pkt)
    ;

  class_<bootloader_conn>("bootloader_conn", init<tty_serial *>())
    .def("reset_target", &bootloader_conn::reset_target)
    .def("check_bootloader", &bootloader_conn::check_bootloader)
    //.def("transaction", &bootloader_conn_transaction)
    .def("chip_erase", &bootloader_conn::chip_erase)
    //.def("upload_page", &bootloader_conn_upload_page)
    //.def("download_page", &bootloader_conn_download_page)
    .def("upload_srec", &bootloader_conn_upload_srec)
    //.def("download_srec", &bootloader_conn_download_srec)
    .def("get_hwid", &bootloader_conn::get_hwid)
    .def("set_hwid", &bootloader_conn::set_hwid)
    ;

  class_<debugentity>("debugentity", no_init)
    .def("incr_verbose", &debugentity_incr_verbose)
    .staticmethod("incr_verbose")
    ;
}
