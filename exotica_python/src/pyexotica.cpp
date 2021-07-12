//
// Copyright (c) 2018-2020, University of Edinburgh, University of Oxford
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
//  * Redistributions of source code must retain the above copyright notice,
//    this list of conditions and the following disclaimer.
//  * Redistributions in binary form must reproduce the above copyright
//    notice, this list of conditions and the following disclaimer in the
//    documentation and/or other materials provided with the distribution.
//  * Neither the name of  nor the names of its contributors may be used to
//    endorse or promote products derived from this software without specific
//    prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.
//

#include <exotica_core/exotica_core.h>
#include <exotica_core/tools/box_qp.h>
#include <exotica_core/tools/box_qp_old.h>
#include <exotica_core/tools/sparse_costs.h>
#ifdef MSGPACK_FOUND
#include <exotica_core/visualization_meshcat.h>
#endif
#include <exotica_core/visualization_moveit.h>
#undef NDEBUG
#include <pybind11/eigen.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include <octomap/OcTree.h>
#include <ros/package.h>

#include <unsupported/Eigen/CXX11/Tensor>

#if PY_MAJOR_VERSION < 3
#define PY_OLDSTANDARD
#endif

#ifndef PY_OLDSTANDARD
bool PyInt_Check(PyObject* value_py) { return PyLong_Check(value_py); }
long PyInt_AsLong(PyObject* value_py) { return PyLong_AsLong(value_py); }
#endif

bool IsPyString(PyObject* value_py)
{
#ifndef PY_OLDSTANDARD
    return PyUnicode_Check(value_py);
#else
    return PyString_Check(value_py) || PyUnicode_Check(value_py);
#endif
}

std::string PyAsStdString(PyObject* value_py)
{
#ifndef PY_OLDSTANDARD
    PyObject* tmp = PyUnicode_AsASCIIString(value_py);
    std::string ret = std::string(PyBytes_AsString(tmp));
    Py_DECREF(tmp);
    return ret;
#else
    return std::string(PyString_AsString(value_py));
#endif
}

PyObject* StdStringAsPy(std::string value)
{
#ifndef PY_OLDSTANDARD
    return PyUnicode_DecodeASCII(value.c_str(), value.size(), "");
#else
    return PyString_FromString(value.c_str());
#endif
}

using namespace exotica;
namespace py = pybind11;

std::map<std::string, Initializer> known_initializers;

PyObject* CreateStringIOObject()
{
#if PY_MAJOR_VERSION <= 2
    PyObject* module = PyImport_ImportModule("StringIO");
    if (!module) ThrowPretty("Can't load StringIO module.");
    PyObject* cls = PyObject_GetAttrString(module, "StringIO");
    if (!cls) ThrowPretty("Can't load StringIO class.");
#else
    PyObject* module = PyImport_ImportModule("io");
    if (!module) ThrowPretty("Can't load io module.");
    PyObject* cls = PyObject_GetAttrString(module, "BytesIO");
    if (!cls) ThrowPretty("Can't load BytesIO class.");
#endif
    PyObject* stringio = PyObject_CallObject(cls, NULL);
    if (!stringio) ThrowPretty("Can't create StringIO object.");
    Py_DECREF(module);
    Py_DECREF(cls);
    return stringio;
}

#define ROS_MESSAGE_WRAPPER(MessageType)                                        \
    namespace pybind11                                                          \
    {                                                                           \
    namespace detail                                                            \
    {                                                                           \
    template <>                                                                 \
    struct type_caster<MessageType>                                             \
    {                                                                           \
    public:                                                                     \
        PYBIND11_TYPE_CASTER(MessageType, _("genpy.Message"));                  \
                                                                                \
        bool load(handle src, bool)                                             \
        {                                                                       \
            PyObject* stringio = CreateStringIOObject();                        \
            if (!stringio) ThrowPretty("Can't create StringIO instance.");      \
            PyObject* result =                                                  \
                PyObject_CallMethod(src.ptr(), "serialize", "O", stringio);     \
            if (!result) ThrowPretty("Can't serialize.");                       \
            result = PyObject_CallMethod(stringio, "getvalue", nullptr);        \
            if (!result) ThrowPretty("Can't get buffer.");                      \
            char* data = PyByteArray_AsString(PyByteArray_FromObject(result));  \
            int len = PyByteArray_Size(result);                                 \
            unsigned char* udata = new unsigned char[len];                      \
            for (int i = 0; i < len; ++i)                                       \
                udata[i] = static_cast<unsigned char>(data[i]);                 \
            ros::serialization::IStream stream(udata, len);                     \
            ros::serialization::deserialize<MessageType>(stream, value);        \
            delete[] udata;                                                     \
            delete[] data;                                                      \
            Py_DECREF(stringio);                                                \
            Py_DECREF(result);                                                  \
            return !PyErr_Occurred();                                           \
        }                                                                       \
                                                                                \
        static handle cast(MessageType src,                                     \
                           return_value_policy /* policy /, handle / parent */) \
        {                                                                       \
            ros::message_traits::DataType<MessageType::Type> type;              \
            ThrowPretty("Can't create python object from message of type '"     \
                        << type.value() << "'!");                               \
        }                                                                       \
    };                                                                          \
    }                                                                           \
    }
#include <moveit_msgs/PlanningScene.h>
#include <moveit_msgs/PlanningSceneWorld.h>
ROS_MESSAGE_WRAPPER(moveit_msgs::PlanningScene);
ROS_MESSAGE_WRAPPER(moveit_msgs::PlanningSceneWorld);

Initializer CreateInitializer(const Initializer& init)
{
    return Initializer(init);
}

std::pair<Initializer, Initializer> LoadFromXML(std::string file_name, const std::string& solver_name = "", const std::string& problem_name = "", bool parsePathAsXML = false)
{
    Initializer solver, problem;
    XMLLoader::Load(file_name, solver, problem, solver_name, problem_name, parsePathAsXML);
    return std::pair<Initializer, Initializer>(solver, problem);
}

void AddInitializers(py::module& module)
{
    py::module inits = module.def_submodule("Initializers", "Initializers for core EXOTica classes.");
    inits.def("Initializer", &CreateInitializer);
    std::vector<Initializer> initializers = Setup::GetInitializers();
    for (const Initializer& i : initializers)
    {
        const std::string full_name = i.GetName();
        const std::string name = full_name.substr(8);  // This removes the prefix "exotica/"
        known_initializers[full_name] = CreateInitializer(i);
        inits.def((name + "Initializer").c_str(), [full_name]() { return CreateInitializer(known_initializers[full_name]); }, (name + "Initializer constructor.").c_str());
    }

    inits.def("load_xml", (Initializer(*)(std::string, bool)) & XMLLoader::Load, "Loads initializer from XML", py::arg("xml"), py::arg("parseAsXMLString") = false);
    inits.def("load_xml_full", &LoadFromXML, "Loads initializer from XML", py::arg("xml"), py::arg("solver_name") = std::string(""), py::arg("problem_name") = std::string(""), py::arg("parseAsXMLString") = false);
}

namespace pybind11
{
namespace detail
{
template <>
struct type_caster<Initializer>
{
public:
    PYBIND11_TYPE_CASTER(Initializer, _("Initializer"));

    bool AddPropertyFromDict(Property& target, PyObject* value_py)
    {
        if (target.GetType() == "std::string" || target.GetType() == GetTypeName(typeid(std::string)))
        {
            target.Set(PyAsStdString(value_py));
            return true;
        }
        else if (target.GetType() == "int")
        {
            if (IsPyString(value_py))
            {
                target.Set(ParseInt(PyAsStdString(value_py)));
                return true;
            }
            else if (PyInt_Check(value_py) || PyLong_Check(value_py))
            {
                target.Set((int)PyInt_AsLong(value_py));
                return true;
            }
        }
        else if (target.GetType() == "long")
        {
            if (IsPyString(value_py))
            {
                target.Set((long)ParseInt(PyAsStdString(value_py)));
                return true;
            }
            else if (PyInt_Check(value_py))
            {
                target.Set(PyInt_AsLong(value_py));
                return true;
            }
            else
            {
                ThrowPretty("to be implemented - please open an issue.");
            }
        }
        else if (target.GetType() == "double")
        {
            if (IsPyString(value_py))
            {
                target.Set(ParseDouble(PyAsStdString(value_py)));
                return true;
            }
            else if (PyFloat_Check(value_py))
            {
                target.Set(PyFloat_AsDouble(value_py));
                return true;
            }
        }
        else if (target.GetType() == "Eigen::Matrix<double, -1, 1, 0, -1, 1>")
        {
            if (IsPyString(value_py))
            {
                target.Set(ParseVector<double, Eigen::Dynamic>(PyAsStdString(value_py)));
            }
            else
            {
                target.Set(py::cast<Eigen::VectorXd>(value_py));
            }
            return true;
        }
        else if (target.GetType() == "Eigen::Matrix<int, -1, 1, 0, -1, 1>")
        {
            if (IsPyString(value_py))
            {
                target.Set(ParseVector<int, Eigen::Dynamic>(PyAsStdString(value_py)));
            }
            else
            {
                target.Set(py::cast<Eigen::VectorXi>(value_py));
            }
            return true;
        }
        else if (target.GetType() == "Eigen::Matrix<double, 2, 1, 0, 2, 1>")
        {
            if (IsPyString(value_py))
            {
                target.Set(ParseVector<double, 2>(PyAsStdString(value_py)));
            }
            else
            {
                target.Set(py::cast<Eigen::Vector2d>(value_py));
            }
            return true;
        }
        else if (target.GetType() == "Eigen::Matrix<double, 3, 1, 0, 3, 1>")
        {
            if (IsPyString(value_py))
            {
                target.Set(ParseVector<double, 3>(PyAsStdString(value_py)));
            }
            else
            {
                target.Set(py::cast<Eigen::Vector3d>(value_py));
            }
            return true;
        }
        else if (target.GetType() == GetTypeName(typeid(std::vector<int>)))
        {
            if (IsPyString(value_py))
            {
                target.Set(ParseIntList(PyAsStdString(value_py)));
            }
            else
            {
                target.Set(py::cast<std::vector<int>>(value_py));
            }
            return true;
        }
        else if (target.GetType() == GetTypeName(typeid(std::vector<std::string>)))
        {
            if (IsPyString(value_py))
            {
                target.Set(ParseList(PyAsStdString(value_py)));
            }
            else
            {
                target.Set(py::cast<std::vector<std::string>>(value_py));
            }
            return true;
        }
        else if (target.GetType() == "bool")
        {
            if (IsPyString(value_py))
            {
                target.Set(ParseBool(PyAsStdString(value_py)));
                return true;
            }
            else if (PyBool_Check(value_py))
            {
                target.Set(PyObject_IsTrue(value_py) == 1);
                return true;
            }
        }
        else if (target.GetType() == "exotica::Initializer")
        {
            if (PyList_Check(value_py))
            {
                Initializer tmp;
                int n = PyList_Size(value_py);
                if (n == 1)
                {
                    if (!PyToInit(PyList_GetItem(value_py, 0), tmp))
                    {
                        WARNING("Could not create initializer :'(");
                        return false;
                    }
                }
                else
                {
                    WARNING("List size is greater than 1 - this should not happen: " << n);
                }
                target.Set(tmp);
            }
            else
            {
                Initializer tmp;
                if (!PyToInit(value_py, tmp))
                {
                    WARNING("Could not convert Python value to exotica::Initializer");
                    return false;
                }
                target.Set(tmp);
            }
            return true;
        }
        else if (target.IsInitializerVectorType())
        {
            if (PyList_Check(value_py))
            {
                int n = PyList_Size(value_py);
                std::vector<Initializer> vec(n);
                for (int i = 0; i < n; ++i)
                {
                    if (!PyToInit(PyList_GetItem(value_py, i), vec[i]))
                    {
                        WARNING("Could not parse initializer in vector of initializers: #" << i);
                        return false;
                    }
                }
                target.Set(vec);
                return true;
            }
            else
            {
                HIGHLIGHT("InitializerVectorType failed PyList_Check");
            }
        }
        else
        {
            HIGHLIGHT("Skipping unsupported type '" << target.GetType() << "'");
        }

        return false;
    }

    bool PyToInit(PyObject* source, Initializer& ret)
    {
        if (!PyTuple_CheckExact(source))
        {
            WARNING_NAMED("PyToInit", "Failed exact tuple check.");
            return false;
        }

        int tuple_size = PyTuple_Size(source);
        if (tuple_size < 1 || tuple_size > 2)
        {
            WARNING_NAMED("PyToInit", "Wrong sized tuple for exotica::Initializer: " << tuple_size);
            return false;
        }

        PyObject* const name_py = PyTuple_GetItem(source, 0);
        if (!IsPyString(name_py))
        {
            WARNING_NAMED("PyToInit", "First element of exotica::Initializer-tuple is not a string.");
            return false;
        }
        const std::string initializer_name = PyAsStdString(name_py);

        const auto& it = known_initializers.find(initializer_name);
        if (it == known_initializers.end())
        {
            HIGHLIGHT("Unknown initializer type '" << initializer_name << "'");
            return false;
        }
        ret = Initializer(it->second);

        if (tuple_size == 2)
        {
            PyObject* const dict = PyTuple_GetItem(source, 1);
            if (!PyDict_Check(dict))
            {
                WARNING_NAMED("PyToInit", "Second element of exotica::Initializer-tuple is not a dict.");
                return false;
            }

            PyObject *key, *value_py;
            Py_ssize_t pos = 0;

            while (PyDict_Next(dict, &pos, &key, &value_py))
            {
                const std::string key_str = PyAsStdString(key);

                if (ret.properties_.count(key_str))
                {
                    if (!AddPropertyFromDict(ret.properties_.at(key_str), value_py))
                    {
                        HIGHLIGHT("Failed to add property '" << key_str << "'");
                        return false;
                    }
                }
                else
                {
                    // 2020-11-04: Replaced the ignoring behaviour with a warning that still adds the property to the initializer.
                    // This resolves issue #719: Sometimes (e.g. for SphereCollision ), we do casting to derived types inside a TaskMap.
                    // This requires having the properties added to the generic Initializer, even if the base initializer does not contain them.
                    // HIGHLIGHT(initializer_name << ": Ignoring property '" << key_str << "'")
                    ret.AddProperty(Property(key_str, false, boost::any(PyAsStdString(value_py))));
                    // WARNING("Adding property '" << key_str << "' even though Initializer type '" << initializer_name << "' does not know this property.");
                }
            }
        }
        return true;
    }

    bool load(handle src, bool)
    {
        return PyToInit(src.ptr(), value);
    }

    static PyObject* InitializerToTuple(const Initializer& src)
    {
        PyObject* dict = PyDict_New();
        for (auto& prop : src.properties_)
        {
            addPropertyToDict(dict, prop.first, prop.second);
        }
        PyObject* name = StdStringAsPy(src.GetName());
        PyObject* tup = PyTuple_Pack(2, name, dict);
        Py_DECREF(dict);
        Py_DECREF(name);
        return tup;
    }

    static void addPropertyToDict(PyObject* dict, const std::string& name, const Property& prop)
    {
        if (prop.GetType() == "std::string" || prop.GetType() == GetTypeName(typeid(std::string)))
        {
            PyDict_SetItemString(dict, name.c_str(), py::cast(boost::any_cast<std::string>(prop.Get())).ptr());
        }
        else if (prop.GetType() == "int")
        {
            PyDict_SetItemString(dict, name.c_str(), py::cast(boost::any_cast<int>(prop.Get())).ptr());
        }
        else if (prop.GetType() == "long")
        {
            PyDict_SetItemString(dict, name.c_str(), py::cast(boost::any_cast<long>(prop.Get())).ptr());
        }
        else if (prop.GetType() == "double")
        {
            PyDict_SetItemString(dict, name.c_str(), py::cast(boost::any_cast<double>(prop.Get())).ptr());
        }
        else if (prop.GetType() == "Eigen::Matrix<double, -1, 1, 0, -1, 1>")
        {
            PyDict_SetItemString(dict, name.c_str(), py::cast(boost::any_cast<Eigen::VectorXd>(prop.Get())).ptr());
        }
        else if (prop.GetType() == "Eigen::Matrix<int, -1, 1, 0, -1, 1>")
        {
            PyDict_SetItemString(dict, name.c_str(), py::cast(boost::any_cast<Eigen::VectorXi>(prop.Get())).ptr());
        }
        else if (prop.GetType() == "Eigen::Matrix<double, 2, 1, 0, 2, 1>")
        {
            PyDict_SetItemString(dict, name.c_str(), py::cast(boost::any_cast<Eigen::Vector2d>(prop.Get())).ptr());
        }
        else if (prop.GetType() == "Eigen::Matrix<double, 3, 1, 0, 3, 1>")
        {
            PyDict_SetItemString(dict, name.c_str(), py::cast(boost::any_cast<Eigen::Vector3d>(prop.Get())).ptr());
        }
        else if (prop.GetType() == GetTypeName(typeid(std::vector<int>)))
        {
            PyDict_SetItemString(dict, name.c_str(), py::cast(boost::any_cast<std::vector<int>>(prop.Get())).ptr());
        }
        else if (prop.GetType() == GetTypeName(typeid(std::vector<std::string>)))
        {
            PyDict_SetItemString(dict, name.c_str(), py::cast(boost::any_cast<std::vector<std::string>>(prop.Get())).ptr());
        }
        else if (prop.GetType() == "bool")
        {
            PyDict_SetItemString(dict, name.c_str(), py::cast(boost::any_cast<bool>(prop.Get())).ptr());
        }
        else if (prop.GetType() == "exotica::Initializer")
        {
            PyObject* init = InitializerToTuple(boost::any_cast<Initializer>(prop.Get()));
            PyDict_SetItemString(dict, name.c_str(), init);
            Py_DECREF(init);
        }
        else if (prop.IsInitializerVectorType())
        {
            PyObject* vec = PyList_New(0);
            for (Initializer& i : boost::any_cast<std::vector<Initializer>>(prop.Get()))
            {
                PyObject* init = InitializerToTuple(i);
                PyList_Append(vec, init);
                Py_DECREF(init);
            }
            PyDict_SetItemString(dict, name.c_str(), vec);
            Py_DECREF(vec);
        }
        else
        {
            HIGHLIGHT("Skipping unsupported type '" << prop.GetType() << "'");
        }
    }

    static handle cast(Initializer src, return_value_policy /* policy */, handle /* parent */)
    {
        return handle(InitializerToTuple(src));
    }
};
}  // namespace detail
}  // namespace pybind11

template <class T>
Hessian array_hessian(
    pybind11::array_t<T> inArray)
{
    // request a buffer descriptor from Python
    pybind11::buffer_info buffer_info = inArray.request();

    // extract data an shape of input array
    T* data = static_cast<T*>(buffer_info.ptr);
    std::vector<ssize_t> shape = buffer_info.shape;

    // wrap ndarray in Eigen::Map:
    // the second template argument is the rank of the tensor and has to be
    // known at compile time
    Eigen::TensorMap<Eigen::Tensor<T, 3, Eigen::RowMajor>> in_tensor(
        data, shape[0], shape[1], shape[2]);

    Hessian hessian = Hessian::Constant(shape[0], Eigen::MatrixXd::Zero(shape[1], shape[2]));
    for (int i = 0; i < shape[0]; i++)
    {
        for (int j = 0; j < shape[1]; j++)
        {
            for (int k = 0; k < shape[2]; k++)
            {
                hessian(i)(j, k) = in_tensor(i, j, k);
            }
        }
    }
    return hessian;
}

template <class T>
py::handle hessian_array(
    Hessian& inp)
{
    std::vector<ssize_t> shape(3);
    shape[0] = inp.rows();
    if (shape[0] > 0)
    {
        shape[1] = inp(0).rows();
        shape[2] = inp(0).cols();
    }
    else
    {
        shape[1] = shape[2] = 0;
    }

    pybind11::array_t<T> array(
        shape,                                                                // shape
        {shape[1] * shape[2] * sizeof(T), shape[2] * sizeof(T), sizeof(T)});  // stride

    Eigen::TensorMap<Eigen::Tensor<T, 3, Eigen::RowMajor>> in_tensor(
        array.mutable_data(), shape[0], shape[1], shape[2]);
    for (int i = 0; i < shape[0]; i++)
    {
        for (int j = 0; j < shape[1]; j++)
        {
            for (int k = 0; k < shape[2]; k++)
            {
                in_tensor(i, j, k) = inp(i)(j, k);
            }
        }
    }
    return array.release();
}

namespace pybind11
{
namespace detail
{
template <>
struct type_caster<Hessian>
{
public:
    /**
         * This macro establishes the name 'inty' in
         * function signatures and declares a local variable
         * 'value' of type inty
         */
    PYBIND11_TYPE_CASTER(Hessian, _("Hessian"));

    /**
         * Conversion part 1 (Python->C++): convert a PyObject into a inty
         * instance or return false upon failure. The second argument
         * indicates whether implicit conversions should be applied.
         */
    bool load(handle src, bool)
    {
        value = array_hessian(py::cast<pybind11::array_t<Hessian::Scalar::Scalar>>(src.ptr()));
        return true;
    }

    /**
         * Conversion part 2 (C++ -> Python): convert an inty instance into
         * a Python object. The second and third arguments are used to
         * indicate the return value policy and parent object (for
         * ``return_value_policy::reference_internal``) and are generally
         * ignored by implicit casters.
         */
    static handle cast(Hessian src, return_value_policy /* policy */, handle /* parent */)
    {
        return hessian_array<Hessian::Scalar::Scalar>(src);
    }
};
}  // namespace detail
}  // namespace pybind11

PYBIND11_MODULE(_pyexotica, module)
{
    module.doc() = "Exotica Python wrapper";

    py::class_<Setup, std::unique_ptr<Setup, py::nodelete>> setup(module, "Setup");
    setup.def(py::init([]() { return Setup::Instance().get(); }));
    setup.def_static("get_solvers", &Setup::GetSolvers, "Returns a list of available solvers.");
    setup.def_static("get_problems", &Setup::GetProblems, "Returns a list of available problems.");
    setup.def_static("get_maps", &Setup::GetMaps, "Returns a list of available task maps.");
    setup.def_static("get_collision_scenes", &Setup::GetCollisionScenes, "Returns a list of available collision scene plug-ins.");
    setup.def_static("get_dynamics_solvers", &Setup::GetDynamicsSolvers, "Returns a list of available dynamics solvers plug-ins.");
    setup.def_static("create_solver", [](const Initializer& init) { return Setup::CreateSolver(init); }, py::return_value_policy::take_ownership);    // "Creates an instance of the solver identified by name parameter.", py::arg("solverType"), py::arg("prependExoticaNamespace"));
    setup.def_static("create_problem", [](const Initializer& init) { return Setup::CreateProblem(init); }, py::return_value_policy::take_ownership);  // "Creates an instance of the problem identified by name parameter.", py::arg("problemType"), py::arg("prependExoticaNamespace"));
    setup.def_static("create_scene", [](const Initializer& init) { return Setup::CreateScene(init); }, py::return_value_policy::take_ownership);
    setup.def_static("create_dynamics_solver", [](const Initializer& init) { return Setup::CreateDynamicsSolver(init); }, py::return_value_policy::take_ownership);
    setup.def_static("print_supported_classes", &Setup::PrintSupportedClasses, "Print a list of available plug-ins sorted by class.");
    setup.def_static("get_initializers", &Setup::GetInitializers, py::return_value_policy::copy, "Returns a list of available initializers with all available parameters/arguments.");
    setup.def_static("get_package_path", &ros::package::getPath, "ROS package path resolution.");
    setup.def_static("init_ros",
                     [](const std::string& name, const bool& anonymous) {
                         int argc = 0;
                         if (anonymous)
                         {
                             ros::init(argc, nullptr, name, ros::init_options::AnonymousName);
                         }
                         else
                         {
                             ros::init(argc, nullptr, name);
                         }
                         Server::InitRos(std::make_shared<ros::NodeHandle>("~"));
                     },
                     "Initializes an internal ROS node for publishing debug information from Exotica (i.e., activates ROS features). Options are setting the name and whether to spawn an anonymous node.",
                     py::arg("name") = "exotica", py::arg("anonymous") = false);
    setup.def_static("load_solver", &XMLLoader::LoadSolver, "Instantiate solver and problem from an XML file containing both a solver and problem initializer.", py::arg("filepath"));
    setup.def_static("load_solver_standalone", &XMLLoader::LoadSolverStandalone, "Instantiate only a solver from an XML file containing solely a solver initializer.", py::arg("filepath"));
    setup.def_static("load_problem", &XMLLoader::LoadProblem, "Instantiate only a problem from an XML file containing solely a problem initializer.", py::arg("filepath"));

    py::module tools = module.def_submodule("Tools");
    tools.def("parse_path", &ParsePath);
    tools.def("parse_bool", &ParseBool);
    tools.def("parse_double", &ParseDouble);
    tools.def("parse_vector", &ParseVector<double, Eigen::Dynamic>);
    tools.def("parse_list", &ParseList);
    tools.def("parse_int", &ParseInt);
    tools.def("parse_int_list", &ParseIntList);
    tools.def("load_obj", [](const std::string& path) { Eigen::VectorXi tri; Eigen::VectorXd vert; LoadOBJ(LoadFile(path), tri, vert); return py::make_tuple(tri, vert); });
    tools.def("load_octree", &LoadOctree);
    tools.def("load_octree_as_shape", &LoadOctreeAsShape);
    tools.def("save_matrix", &SaveMatrix);
    tools.def("VectorTransform", &Eigen::VectorTransform);
    tools.def("IdentityTransform", &Eigen::IdentityTransform);
    tools.def("load_file", &LoadFile);
    tools.def("path_exists", &PathExists);
    tools.def("create_composite_trajectory", [](Eigen::MatrixXdRefConst data, double radius) {
        return Trajectory(data, radius).ToString();
    },
              py::arg("data"), py::arg("max_radius") = 1.0);

    py::module sparse_costs = tools.def_submodule("SparseCosts")
                                  .def("huber_cost", &huber_cost)
                                  .def("huber_jacobian", &huber_jacobian)
                                  .def("huber_hessian", &huber_hessian)
                                  .def("smooth_l1_cost", &smooth_l1_cost)
                                  .def("smooth_l1_jacobian", &smooth_l1_jacobian)
                                  .def("smooth_l1_hessian", &smooth_l1_hessian)
                                  .def("pseudo_huber_cost", &pseudo_huber_cost)
                                  .def("pseudo_huber_jacobian", &pseudo_huber_jacobian)
                                  .def("pseudo_huber_hessian", &pseudo_huber_hessian);

    py::class_<Timer, std::shared_ptr<Timer>> timer(module, "Timer");
    timer.def(py::init());
    timer.def("reset", &Timer::Reset);
    timer.def("get_duration", &Timer::GetDuration);

    py::class_<Object, std::shared_ptr<Object>> object(module, "Object");
    object.def_property_readonly("type", &Object::type, "Object type");
    object.def_property_readonly("name", &Object::GetObjectName, "Object name");
    object.def("__repr__", &Object::Print, "String representation of the object", py::arg("prepend") = std::string(""));
    object.def_readwrite("namespace", &Object::ns_);
    object.def_readwrite("debug_mode", &Object::debug_);

    py::enum_<TerminationCriterion>(module, "TerminationCriterion")
        .value("NotStarted", TerminationCriterion::NotStarted)
        .value("IterationLimit", TerminationCriterion::IterationLimit)
        .value("BacktrackIterationLimit", TerminationCriterion::BacktrackIterationLimit)
        .value("StepTolerance", TerminationCriterion::StepTolerance)
        .value("FunctionTolerance", TerminationCriterion::FunctionTolerance)
        .value("GradientTolerance", TerminationCriterion::GradientTolerance)
        .value("Divergence", TerminationCriterion::Divergence)
        .value("UserDefined", TerminationCriterion::UserDefined)
        .value("Convergence", TerminationCriterion::Convergence)
        .export_values();

    py::enum_<RotationType>(module, "RotationType")
        .value("Quaternion", RotationType::QUATERNION)
        .value("RPY", RotationType::RPY)
        .value("ZYZ", RotationType::ZYZ)
        .value("ZYX", RotationType::ZYX)
        .value("AngleAxis", RotationType::ANGLE_AXIS)
        .value("Matrix", RotationType::MATRIX)
        .export_values();

    py::enum_<BaseType>(module, "BaseType")
        .value("Fixed", BaseType::FIXED)
        .value("Floating", BaseType::FLOATING)
        .value("Planar", BaseType::PLANAR)
        .export_values();

    py::class_<KDL::Frame> kdl_frame(module, "KDLFrame");
    kdl_frame.def(py::init());
    kdl_frame.def(py::init([](Eigen::MatrixXd other) { return GetFrameFromMatrix(other); }));
    kdl_frame.def(py::init([](Eigen::VectorXd other) { return GetFrame(other); }));
    kdl_frame.def(py::init([](const KDL::Frame& other) { return KDL::Frame(other); }));
    kdl_frame.def("__repr__", [](KDL::Frame* me) { return "KDL::Frame " + ToString(*me); });
    kdl_frame.def("get_rpy", [](KDL::Frame* me) { return GetRotationAsVector(*me, RotationType::RPY); });
    kdl_frame.def("get_zyz", [](KDL::Frame* me) { return GetRotationAsVector(*me, RotationType::ZYZ); });
    kdl_frame.def("get_zyx", [](KDL::Frame* me) { return GetRotationAsVector(*me, RotationType::ZYX); });
    kdl_frame.def("get_angle_axis", [](KDL::Frame* me) { return GetRotationAsVector(*me, RotationType::ANGLE_AXIS); });
    kdl_frame.def("get_quaternion", [](KDL::Frame* me) { return GetRotationAsVector(*me, RotationType::QUATERNION); });
    kdl_frame.def("get_translation", [](KDL::Frame* me) { Eigen::Vector3d tmp; for (int i = 0; i < 3; ++i) { tmp[i] = me->p.data[i]; } return tmp; });
    kdl_frame.def("get_translation_and_rpy", [](KDL::Frame* me) { return GetFrameAsVector(*me, RotationType::RPY); });
    kdl_frame.def("get_translation_and_zyz", [](KDL::Frame* me) { return GetFrameAsVector(*me, RotationType::ZYZ); });
    kdl_frame.def("get_translation_and_zyx", [](KDL::Frame* me) { return GetFrameAsVector(*me, RotationType::ZYX); });
    kdl_frame.def("get_translation_and_angle_axis", [](KDL::Frame* me) { return GetFrameAsVector(*me, RotationType::ANGLE_AXIS); });
    kdl_frame.def("get_translation_and_quaternion", [](KDL::Frame* me) { return GetFrameAsVector(*me, RotationType::QUATERNION); });
    kdl_frame.def("get_frame", [](KDL::Frame* me) { return GetFrame(*me); });
    kdl_frame.def("inverse", (KDL::Frame(KDL::Frame::*)() const) & KDL::Frame::Inverse);
    kdl_frame.def("__mul__", [](const KDL::Frame& A, const KDL::Frame& B) { return A * B; }, py::is_operator());
    kdl_frame.def_readwrite("p", &KDL::Frame::p);
    kdl_frame.def_static("interpolate", [](KDL::Frame* A, KDL::Frame* B, double alpha) { return KDL::addDelta(*A, KDL::diff(*A, *B) * alpha); });
    kdl_frame.def_static("diff", [](KDL::Frame* A, KDL::Frame* B) { Eigen::VectorXd ret(6); KDL::Twist t = KDL::diff(*A, *B); for(int i=0; i<6; ++i) ret(i) = t[i]; return ret; });
    py::implicitly_convertible<Eigen::MatrixXd, KDL::Frame>();
    py::implicitly_convertible<Eigen::VectorXd, KDL::Frame>();

    py::class_<KDL::Vector>(module, "KDLVector")
        .def(py::init())
        .def(py::init<double, double, double>(),
             py::arg("x") = 0,
             py::arg("y") = 0,
             py::arg("z") = 0)
        .def("x", [](KDL::Vector& v) -> double& { return v[0]; })
        .def("y", [](KDL::Vector& v) -> double& { return v[1]; })
        .def("z", [](KDL::Vector& v) -> double& { return v[2]; })
        .def_static("Zero", &KDL::Vector::Zero);

    py::class_<KDL::RotationalInertia>(module, "KDLRotationalInertia")
        .def(py::init<double, double, double, double, double, double>(),
             py::arg("Ixx") = 0,
             py::arg("Iyy") = 0,
             py::arg("Izz") = 0,
             py::arg("Ixy") = 0,
             py::arg("Ixz") = 0,
             py::arg("Iyz") = 0)
        .def_static("Zero", &KDL::RotationalInertia::Zero);

    py::class_<KDL::RigidBodyInertia>(module, "KDLRigidBodyInertia")
        .def(py::init<double, KDL::Vector&, KDL::RotationalInertia&>(),
             py::arg("m") = 0,
             py::arg("oc") = KDL::Vector::Zero(),
             py::arg("Ic") = KDL::RotationalInertia::Zero())
        .def_static("Zero", &KDL::RigidBodyInertia::Zero);

    py::class_<TaskMap, std::shared_ptr<TaskMap>, Object>(module, "TaskMap")
        .def_readonly("id", &TaskMap::id)
        .def_readonly("start", &TaskMap::start)
        .def_readonly("length", &TaskMap::length)
        .def_readonly("startJ", &TaskMap::start_jacobian)
        .def_readonly("lengthJ", &TaskMap::length_jacobian)
        .def("task_space_dim", (int (TaskMap::*)()) & TaskMap::TaskSpaceDim)
        .def("task_space_jacobian_dim", &TaskMap::TaskSpaceJacobianDim);

    py::class_<TaskIndexing>(module, "TaskIndexing")
        .def_readonly("id", &TaskIndexing::id)
        .def_readonly("start", &TaskIndexing::start)
        .def_readonly("length", &TaskIndexing::length)
        .def_readonly("startJ", &TaskIndexing::start_jacobian)
        .def_readonly("lengthJ", &TaskIndexing::length_jacobian);

    py::class_<TimeIndexedTask, std::shared_ptr<TimeIndexedTask>>(module, "TimeIndexedTask")
        .def_readonly("indexing", &TimeIndexedTask::indexing)
        .def_readonly("length_Phi", &TimeIndexedTask::length_Phi)
        .def_readonly("length_jacobian", &TimeIndexedTask::length_jacobian)
        .def_readonly("num_tasks", &TimeIndexedTask::num_tasks)
        .def_readonly("y", &TimeIndexedTask::y)
        .def_readonly("ydiff", &TimeIndexedTask::ydiff)
        .def_readonly("Phi", &TimeIndexedTask::Phi)
        .def_readonly("rho", &TimeIndexedTask::rho)
        .def_readonly("hessian", &TimeIndexedTask::hessian)        // Kinematic
        .def_readonly("jacobian", &TimeIndexedTask::jacobian)      // Kinematic
        .def_readonly("dPhi_dx", &TimeIndexedTask::dPhi_dx)        // Dynamic
        .def_readonly("dPhi_du", &TimeIndexedTask::dPhi_du)        // Dynamic
        .def_readonly("ddPhi_ddx", &TimeIndexedTask::ddPhi_ddx)    // Dynamic
        .def_readonly("ddPhi_ddu", &TimeIndexedTask::ddPhi_ddu)    // Dynamic
        .def_readonly("ddPhi_dxdu", &TimeIndexedTask::ddPhi_dxdu)  // Dynamic
        .def_readonly("S", &TimeIndexedTask::S)
        .def_readonly("T", &TimeIndexedTask::T)
        .def_readonly("tasks", &TimeIndexedTask::tasks)
        .def_readonly("task_maps", &TimeIndexedTask::task_maps)
        .def("set_goal", &TimeIndexedTask::SetGoal)
        .def("get_goal", &TimeIndexedTask::GetGoal)
        .def("set_rho", &TimeIndexedTask::SetRho)
        .def("get_rho", &TimeIndexedTask::GetRho)
        .def("get_task_error", &TimeIndexedTask::GetTaskError)
        .def("get_S", &TimeIndexedTask::GetS);

    py::class_<EndPoseTask, std::shared_ptr<EndPoseTask>>(module, "EndPoseTask")
        .def_readonly("length_Phi", &EndPoseTask::length_Phi)
        .def_readonly("length_jacobian", &EndPoseTask::length_jacobian)
        .def_readonly("num_tasks", &EndPoseTask::num_tasks)
        .def_readonly("y", &EndPoseTask::y)
        .def_readonly("ydiff", &EndPoseTask::ydiff)
        .def_readonly("Phi", &EndPoseTask::Phi)
        // .def_readonly("hessian", &EndPoseTask::hessian)
        .def_readonly("jacobian", &EndPoseTask::jacobian)
        .def_readonly("S", &EndPoseTask::S)
        .def_readonly("tasks", &EndPoseTask::tasks)
        .def_readonly("task_maps", &EndPoseTask::task_maps)
        .def("get_S", &EndPoseTask::GetS)
        .def("get_task_error", &EndPoseTask::GetTaskError)
        .def("get_task_jacobian", &EndPoseTask::GetTaskJacobian)
        .def("set_goal", &EndPoseTask::SetGoal)
        .def("get_goal", &EndPoseTask::GetGoal)
        .def("set_rho", &EndPoseTask::SetRho)
        .def("get_rho", &EndPoseTask::GetRho);

    py::class_<SamplingTask, std::shared_ptr<SamplingTask>>(module, "SamplingTask")
        .def_readonly("length_Phi", &SamplingTask::length_Phi)
        .def_readonly("length_jacobian", &SamplingTask::length_jacobian)
        .def_readonly("num_tasks", &SamplingTask::num_tasks)
        .def_readonly("y", &SamplingTask::y)
        .def_readonly("ydiff", &SamplingTask::ydiff)
        .def_readonly("Phi", &SamplingTask::Phi)
        .def_readonly("S", &SamplingTask::S)
        .def_readonly("tasks", &SamplingTask::tasks)
        .def_readonly("task_maps", &SamplingTask::task_maps)
        .def("set_goal", &SamplingTask::SetGoal)
        .def("get_goal", &SamplingTask::GetGoal)
        .def("set_rho", &SamplingTask::SetRho)
        .def("get_rho", &SamplingTask::GetRho);

    py::class_<TaskSpaceVector, std::shared_ptr<TaskSpaceVector>> task_space_vector(module, "TaskSpaceVector");
    task_space_vector.def("set_zero", &TaskSpaceVector::SetZero);
    task_space_vector.def_readonly("data", &TaskSpaceVector::data);
    task_space_vector.def("__sub__", &TaskSpaceVector::operator-, py::is_operator());
    task_space_vector.def("__repr__", [](TaskSpaceVector* instance) { return ((std::ostringstream&)(std::ostringstream("") << "TaskSpaceVector (" << instance->data.transpose() << ")")).str(); });

    py::class_<MotionSolver, std::shared_ptr<MotionSolver>, Object> motion_solver(module, "MotionSolver");
    motion_solver.def_property("max_iterations", &MotionSolver::GetNumberOfMaxIterations, &MotionSolver::SetNumberOfMaxIterations);
    motion_solver.def("get_planning_time", &MotionSolver::GetPlanningTime);
    motion_solver.def("specify_problem", &MotionSolver::SpecifyProblem, "Assign problem to the solver", py::arg("planning_problem"));
    motion_solver.def(
        "solve", [](std::shared_ptr<MotionSolver> sol) {
            Eigen::MatrixXd ret;
            sol->Solve(ret);
            return ret;
        },
        "Solve the problem");
    motion_solver.def("get_problem", &MotionSolver::GetProblem);

    py::class_<FeedbackMotionSolver, std::shared_ptr<FeedbackMotionSolver>, MotionSolver> feedback_motion_solver(module, "FeedbackMotionSolver");
    feedback_motion_solver.def("get_feedback_control", &FeedbackMotionSolver::GetFeedbackControl);

    py::class_<PlanningProblem, std::shared_ptr<PlanningProblem>, Object>(module, "PlanningProblem")
        .def("get_tasks", &PlanningProblem::GetTasks, py::return_value_policy::reference_internal)
        .def("get_task_maps", &PlanningProblem::GetTaskMaps, py::return_value_policy::reference_internal)
        .def("get_scene", &PlanningProblem::GetScene, py::return_value_policy::reference_internal)
        .def("__repr__", &PlanningProblem::Print, "String representation of the object", py::arg("prepend") = std::string(""))
        .def_readonly("N", &PlanningProblem::N)
        .def_property_readonly("num_positions", &PlanningProblem::get_num_positions)    // deprecated
        .def_property_readonly("num_velocities", &PlanningProblem::get_num_velocities)  // deprecated
        .def_property_readonly("num_controls", &PlanningProblem::get_num_controls)      // deprecated
        .def_property("start_state", &PlanningProblem::GetStartState, &PlanningProblem::SetStartState)
        .def_property("start_time", &PlanningProblem::GetStartTime, &PlanningProblem::SetStartTime)
        .def("get_number_of_problem_updates", &PlanningProblem::GetNumberOfProblemUpdates)
        .def("reset_number_of_problem_updates", &PlanningProblem::ResetNumberOfProblemUpdates)
        .def("get_cost_evolution", (std::pair<std::vector<double>, std::vector<double>>(PlanningProblem::*)() const) & PlanningProblem::GetCostEvolution)
        .def("get_number_of_iterations", &PlanningProblem::GetNumberOfIterations)
        .def("pre_update", &PlanningProblem::PreUpdate)
        .def("is_valid", &PlanningProblem::IsValid)
        .def("apply_start_state", &PlanningProblem::ApplyStartState)
        .def_readonly("termination_criterion", &PlanningProblem::termination_criterion);

    // Problem types
    py::module prob = module.def_submodule("Problems", "Problem types");

    py::class_<UnconstrainedTimeIndexedProblem, std::shared_ptr<UnconstrainedTimeIndexedProblem>, PlanningProblem> unconstrained_time_indexed_problem(prob, "UnconstrainedTimeIndexedProblem");
    unconstrained_time_indexed_problem.def("get_duration", &UnconstrainedTimeIndexedProblem::GetDuration);
    unconstrained_time_indexed_problem.def("update", (void (UnconstrainedTimeIndexedProblem::*)(Eigen::VectorXdRefConst, int)) & UnconstrainedTimeIndexedProblem::Update);
    unconstrained_time_indexed_problem.def("update", (void (UnconstrainedTimeIndexedProblem::*)(Eigen::VectorXdRefConst)) & UnconstrainedTimeIndexedProblem::Update);
    unconstrained_time_indexed_problem.def("set_goal", &UnconstrainedTimeIndexedProblem::SetGoal);
    unconstrained_time_indexed_problem.def("set_rho", &UnconstrainedTimeIndexedProblem::SetRho);
    unconstrained_time_indexed_problem.def("get_goal", &UnconstrainedTimeIndexedProblem::GetGoal);
    unconstrained_time_indexed_problem.def("get_rho", &UnconstrainedTimeIndexedProblem::GetRho);
    unconstrained_time_indexed_problem.def_property("tau", &UnconstrainedTimeIndexedProblem::GetTau, &UnconstrainedTimeIndexedProblem::SetTau);
    unconstrained_time_indexed_problem.def_readwrite("W", &UnconstrainedTimeIndexedProblem::W);
    unconstrained_time_indexed_problem.def_property("initial_trajectory", &UnconstrainedTimeIndexedProblem::GetInitialTrajectory, &UnconstrainedTimeIndexedProblem::SetInitialTrajectory);
    unconstrained_time_indexed_problem.def_property("T", &UnconstrainedTimeIndexedProblem::GetT, &UnconstrainedTimeIndexedProblem::SetT);
    unconstrained_time_indexed_problem.def_readonly("length_Phi", &UnconstrainedTimeIndexedProblem::length_Phi);
    unconstrained_time_indexed_problem.def_readonly("length_jacobian", &UnconstrainedTimeIndexedProblem::length_jacobian);
    unconstrained_time_indexed_problem.def_readonly("num_tasks", &UnconstrainedTimeIndexedProblem::num_tasks);
    unconstrained_time_indexed_problem.def_readonly("Phi", &UnconstrainedTimeIndexedProblem::Phi);
    unconstrained_time_indexed_problem.def_readonly("jacobian", &UnconstrainedTimeIndexedProblem::jacobian);
    unconstrained_time_indexed_problem.def("get_scalar_task_cost", &UnconstrainedTimeIndexedProblem::GetScalarTaskCost);
    unconstrained_time_indexed_problem.def("get_scalar_task_jacobian", &UnconstrainedTimeIndexedProblem::GetScalarTaskJacobian);
    unconstrained_time_indexed_problem.def("get_scalar_transition_cost", &UnconstrainedTimeIndexedProblem::GetScalarTransitionCost);
    unconstrained_time_indexed_problem.def("get_scalar_transition_jacobian", &UnconstrainedTimeIndexedProblem::GetScalarTransitionJacobian);
    unconstrained_time_indexed_problem.def_readonly("cost", &UnconstrainedTimeIndexedProblem::cost);

    py::class_<TimeIndexedProblem, std::shared_ptr<TimeIndexedProblem>, PlanningProblem> time_indexed_problem(prob, "TimeIndexedProblem");
    time_indexed_problem.def("get_duration", &TimeIndexedProblem::GetDuration);
    time_indexed_problem.def("update", (void (TimeIndexedProblem::*)(Eigen::VectorXdRefConst, int)) & TimeIndexedProblem::Update);
    time_indexed_problem.def("update", (void (TimeIndexedProblem::*)(Eigen::VectorXdRefConst)) & TimeIndexedProblem::Update);
    time_indexed_problem.def("set_goal", &TimeIndexedProblem::SetGoal);
    time_indexed_problem.def("set_rho", &TimeIndexedProblem::SetRho);
    time_indexed_problem.def("get_goal", &TimeIndexedProblem::GetGoal);
    time_indexed_problem.def("get_rho", &TimeIndexedProblem::GetRho);
    time_indexed_problem.def("set_goal_eq", &TimeIndexedProblem::SetGoalEQ);
    time_indexed_problem.def("set_rho_eq", &TimeIndexedProblem::SetRhoEQ);
    time_indexed_problem.def("get_goal_eq", &TimeIndexedProblem::GetGoalEQ);
    time_indexed_problem.def("get_rho_eq", &TimeIndexedProblem::GetRhoEQ);
    time_indexed_problem.def("set_goal_neq", &TimeIndexedProblem::SetGoalNEQ);
    time_indexed_problem.def("set_rho_neq", &TimeIndexedProblem::SetRhoNEQ);
    time_indexed_problem.def("get_goal_neq", &TimeIndexedProblem::GetGoalNEQ);
    time_indexed_problem.def("get_rho_neq", &TimeIndexedProblem::GetRhoNEQ);
    time_indexed_problem.def_property("tau", &TimeIndexedProblem::GetTau, &TimeIndexedProblem::SetTau);
    time_indexed_problem.def_property("q_dot_max", &TimeIndexedProblem::GetJointVelocityLimits, &TimeIndexedProblem::SetJointVelocityLimits);
    time_indexed_problem.def_readwrite("W", &TimeIndexedProblem::W);
    time_indexed_problem.def_readwrite("use_bounds", &TimeIndexedProblem::use_bounds);
    time_indexed_problem.def_property("initial_trajectory", &TimeIndexedProblem::GetInitialTrajectory, &TimeIndexedProblem::SetInitialTrajectory);
    time_indexed_problem.def_property("T", &TimeIndexedProblem::GetT, &TimeIndexedProblem::SetT);
    time_indexed_problem.def_readonly("length_Phi", &TimeIndexedProblem::length_Phi);
    time_indexed_problem.def_readonly("length_jacobian", &TimeIndexedProblem::length_jacobian);
    time_indexed_problem.def_readonly("num_tasks", &TimeIndexedProblem::num_tasks);
    time_indexed_problem.def_readonly("Phi", &TimeIndexedProblem::Phi);
    time_indexed_problem.def_readonly("jacobian", &TimeIndexedProblem::jacobian);
    time_indexed_problem.def("get_cost", &TimeIndexedProblem::GetCost);
    time_indexed_problem.def("get_cost_jacobian", &TimeIndexedProblem::GetCostJacobian);
    time_indexed_problem.def("get_scalar_task_cost", &TimeIndexedProblem::GetScalarTaskCost);
    time_indexed_problem.def("get_scalar_task_jacobian", &TimeIndexedProblem::GetScalarTaskJacobian);
    time_indexed_problem.def("get_scalar_transition_cost", &TimeIndexedProblem::GetScalarTransitionCost);
    time_indexed_problem.def("get_scalar_transition_jacobian", &TimeIndexedProblem::GetScalarTransitionJacobian);
    time_indexed_problem.def("get_equality", (Eigen::VectorXd(TimeIndexedProblem::*)() const) & TimeIndexedProblem::GetEquality);
    time_indexed_problem.def("get_equality", (Eigen::VectorXd(TimeIndexedProblem::*)(int) const) & TimeIndexedProblem::GetEquality);
    time_indexed_problem.def("get_equality_jacobian", (Eigen::SparseMatrix<double>(TimeIndexedProblem::*)() const) & TimeIndexedProblem::GetEqualityJacobian);
    time_indexed_problem.def("get_equality_jacobian", (Eigen::MatrixXd(TimeIndexedProblem::*)(int) const) & TimeIndexedProblem::GetEqualityJacobian);
    time_indexed_problem.def("get_inequality", (Eigen::VectorXd(TimeIndexedProblem::*)() const) & TimeIndexedProblem::GetInequality);
    time_indexed_problem.def("get_inequality", (Eigen::VectorXd(TimeIndexedProblem::*)(int) const) & TimeIndexedProblem::GetInequality);
    time_indexed_problem.def("get_inequality_jacobian", (Eigen::SparseMatrix<double>(TimeIndexedProblem::*)() const) & TimeIndexedProblem::GetInequalityJacobian);
    time_indexed_problem.def("get_inequality_jacobian", (Eigen::MatrixXd(TimeIndexedProblem::*)(int) const) & TimeIndexedProblem::GetInequalityJacobian);
    time_indexed_problem.def("get_bounds", &TimeIndexedProblem::GetBounds);
    time_indexed_problem.def("get_joint_velocity_limits", &TimeIndexedProblem::GetJointVelocityLimits);
    time_indexed_problem.def_readonly("cost", &TimeIndexedProblem::cost);
    time_indexed_problem.def_readonly("inequality", &TimeIndexedProblem::inequality);
    time_indexed_problem.def_readonly("equality", &TimeIndexedProblem::equality);

    py::class_<BoundedTimeIndexedProblem, std::shared_ptr<BoundedTimeIndexedProblem>, PlanningProblem> bounded_time_indexed_problem(prob, "BoundedTimeIndexedProblem");
    bounded_time_indexed_problem.def("get_duration", &BoundedTimeIndexedProblem::GetDuration);
    bounded_time_indexed_problem.def("update", (void (BoundedTimeIndexedProblem::*)(Eigen::VectorXdRefConst, int)) & BoundedTimeIndexedProblem::Update);
    bounded_time_indexed_problem.def("update", (void (BoundedTimeIndexedProblem::*)(Eigen::VectorXdRefConst)) & BoundedTimeIndexedProblem::Update);
    bounded_time_indexed_problem.def("set_goal", &BoundedTimeIndexedProblem::SetGoal);
    bounded_time_indexed_problem.def("set_rho", &BoundedTimeIndexedProblem::SetRho);
    bounded_time_indexed_problem.def("get_goal", &BoundedTimeIndexedProblem::GetGoal);
    bounded_time_indexed_problem.def("get_rho", &BoundedTimeIndexedProblem::GetRho);
    bounded_time_indexed_problem.def_property("tau", &BoundedTimeIndexedProblem::GetTau, &BoundedTimeIndexedProblem::SetTau);
    bounded_time_indexed_problem.def_readwrite("W", &BoundedTimeIndexedProblem::W);
    bounded_time_indexed_problem.def_property("initial_trajectory", &BoundedTimeIndexedProblem::GetInitialTrajectory, &BoundedTimeIndexedProblem::SetInitialTrajectory);
    bounded_time_indexed_problem.def_property("T", &BoundedTimeIndexedProblem::GetT, &BoundedTimeIndexedProblem::SetT);
    bounded_time_indexed_problem.def_readonly("length_Phi", &BoundedTimeIndexedProblem::length_Phi);
    bounded_time_indexed_problem.def_readonly("length_jacobian", &BoundedTimeIndexedProblem::length_jacobian);
    bounded_time_indexed_problem.def_readonly("num_tasks", &BoundedTimeIndexedProblem::num_tasks);
    bounded_time_indexed_problem.def_readonly("Phi", &BoundedTimeIndexedProblem::Phi);
    bounded_time_indexed_problem.def_readonly("jacobian", &BoundedTimeIndexedProblem::jacobian);
    bounded_time_indexed_problem.def("get_scalar_task_cost", &BoundedTimeIndexedProblem::GetScalarTaskCost);
    bounded_time_indexed_problem.def("get_scalar_task_jacobian", &BoundedTimeIndexedProblem::GetScalarTaskJacobian);
    bounded_time_indexed_problem.def("get_scalar_transition_cost", &BoundedTimeIndexedProblem::GetScalarTransitionCost);
    bounded_time_indexed_problem.def("get_scalar_transition_jacobian", &BoundedTimeIndexedProblem::GetScalarTransitionJacobian);
    bounded_time_indexed_problem.def("get_bounds", &BoundedTimeIndexedProblem::GetBounds);
    bounded_time_indexed_problem.def_readonly("cost", &BoundedTimeIndexedProblem::cost);

    py::class_<UnconstrainedEndPoseProblem, std::shared_ptr<UnconstrainedEndPoseProblem>, PlanningProblem> unconstrained_end_pose_problem(prob, "UnconstrainedEndPoseProblem");
    unconstrained_end_pose_problem.def("update", &UnconstrainedEndPoseProblem::Update);
    unconstrained_end_pose_problem.def("set_goal", &UnconstrainedEndPoseProblem::SetGoal);
    unconstrained_end_pose_problem.def("set_rho", &UnconstrainedEndPoseProblem::SetRho);
    unconstrained_end_pose_problem.def("get_goal", &UnconstrainedEndPoseProblem::GetGoal);
    unconstrained_end_pose_problem.def("get_rho", &UnconstrainedEndPoseProblem::GetRho);
    unconstrained_end_pose_problem.def_readwrite("W", &UnconstrainedEndPoseProblem::W);
    unconstrained_end_pose_problem.def_readonly("length_Phi", &UnconstrainedEndPoseProblem::length_Phi);
    unconstrained_end_pose_problem.def_readonly("length_jacobian", &UnconstrainedEndPoseProblem::length_jacobian);
    unconstrained_end_pose_problem.def_readonly("num_tasks", &UnconstrainedEndPoseProblem::num_tasks);
    unconstrained_end_pose_problem.def_readonly("Phi", &UnconstrainedEndPoseProblem::Phi);
    unconstrained_end_pose_problem.def_readonly("jacobian", &UnconstrainedEndPoseProblem::jacobian);
    unconstrained_end_pose_problem.def_property_readonly("ydiff", [](UnconstrainedEndPoseProblem* prob) { return prob->cost.ydiff; });
    unconstrained_end_pose_problem.def_property("q_nominal", &UnconstrainedEndPoseProblem::GetNominalPose, &UnconstrainedEndPoseProblem::SetNominalPose);
    unconstrained_end_pose_problem.def("get_scalar_cost", &UnconstrainedEndPoseProblem::GetScalarCost);
    unconstrained_end_pose_problem.def("get_scalar_jacobian", &UnconstrainedEndPoseProblem::GetScalarJacobian);
    unconstrained_end_pose_problem.def("get_scalar_task_cost", &UnconstrainedEndPoseProblem::GetScalarTaskCost);
    unconstrained_end_pose_problem.def_readonly("cost", &UnconstrainedEndPoseProblem::cost);

    py::class_<EndPoseProblem, std::shared_ptr<EndPoseProblem>, PlanningProblem> end_pose_problem(prob, "EndPoseProblem");
    end_pose_problem.def("update", &EndPoseProblem::Update);
    end_pose_problem.def("pre_update", &EndPoseProblem::PreUpdate);
    end_pose_problem.def("set_goal", &EndPoseProblem::SetGoal);
    end_pose_problem.def("set_rho", &EndPoseProblem::SetRho);
    end_pose_problem.def("get_goal", &EndPoseProblem::GetGoal);
    end_pose_problem.def("get_rho", &EndPoseProblem::GetRho);
    end_pose_problem.def("set_goal_eq", &EndPoseProblem::SetGoalEQ);
    end_pose_problem.def("set_rho_eq", &EndPoseProblem::SetRhoEQ);
    end_pose_problem.def("get_goal_eq", &EndPoseProblem::GetGoalEQ);
    end_pose_problem.def("get_rho_eq", &EndPoseProblem::GetRhoEQ);
    end_pose_problem.def("set_goal_neq", &EndPoseProblem::SetGoalNEQ);
    end_pose_problem.def("set_rho_neq", &EndPoseProblem::SetRhoNEQ);
    end_pose_problem.def("get_goal_neq", &EndPoseProblem::GetGoalNEQ);
    end_pose_problem.def("get_rho_neq", &EndPoseProblem::GetRhoNEQ);
    end_pose_problem.def_readwrite("W", &EndPoseProblem::W);
    end_pose_problem.def_readwrite("use_bounds", &EndPoseProblem::use_bounds);
    end_pose_problem.def_readonly("length_Phi", &EndPoseProblem::length_Phi);
    end_pose_problem.def_readonly("length_jacobian", &EndPoseProblem::length_jacobian);
    end_pose_problem.def_readonly("num_tasks", &EndPoseProblem::num_tasks);
    end_pose_problem.def_readonly("Phi", &EndPoseProblem::Phi);
    end_pose_problem.def_readonly("jacobian", &EndPoseProblem::jacobian);
    end_pose_problem.def("get_scalar_cost", &EndPoseProblem::GetScalarCost);
    end_pose_problem.def("get_scalar_jacobian", &EndPoseProblem::GetScalarJacobian);
    end_pose_problem.def("get_scalar_task_cost", &EndPoseProblem::GetScalarTaskCost);
    end_pose_problem.def("get_equality", &EndPoseProblem::GetEquality);
    end_pose_problem.def("get_equality_jacobian", &EndPoseProblem::GetEqualityJacobian);
    end_pose_problem.def("get_inequality", &EndPoseProblem::GetInequality);
    end_pose_problem.def("get_inequality_jacobian", &EndPoseProblem::GetInequalityJacobian);
    end_pose_problem.def("get_bounds", &EndPoseProblem::GetBounds);
    end_pose_problem.def_readonly("cost", &EndPoseProblem::cost);
    end_pose_problem.def_readonly("inequality", &EndPoseProblem::inequality);
    end_pose_problem.def_readonly("equality", &EndPoseProblem::equality);

    py::class_<BoundedEndPoseProblem, std::shared_ptr<BoundedEndPoseProblem>, PlanningProblem> bounded_end_pose_problem(prob, "BoundedEndPoseProblem");
    bounded_end_pose_problem.def("update", &BoundedEndPoseProblem::Update);
    bounded_end_pose_problem.def("set_goal", &BoundedEndPoseProblem::SetGoal);
    bounded_end_pose_problem.def("set_rho", &BoundedEndPoseProblem::SetRho);
    bounded_end_pose_problem.def("get_goal", &BoundedEndPoseProblem::GetGoal);
    bounded_end_pose_problem.def("get_rho", &BoundedEndPoseProblem::GetRho);
    bounded_end_pose_problem.def_readwrite("W", &BoundedEndPoseProblem::W);
    bounded_end_pose_problem.def_readonly("length_Phi", &BoundedEndPoseProblem::length_Phi);
    bounded_end_pose_problem.def_readonly("length_jacobian", &BoundedEndPoseProblem::length_jacobian);
    bounded_end_pose_problem.def_readonly("num_tasks", &BoundedEndPoseProblem::num_tasks);
    bounded_end_pose_problem.def_readonly("Phi", &BoundedEndPoseProblem::Phi);
    bounded_end_pose_problem.def_readonly("jacobian", &BoundedEndPoseProblem::jacobian);
    bounded_end_pose_problem.def("get_scalar_cost", &BoundedEndPoseProblem::GetScalarCost);
    bounded_end_pose_problem.def("get_scalar_jacobian", &BoundedEndPoseProblem::GetScalarJacobian);
    bounded_end_pose_problem.def("get_scalar_task_cost", &BoundedEndPoseProblem::GetScalarTaskCost);
    bounded_end_pose_problem.def("get_bounds", &BoundedEndPoseProblem::GetBounds);
    bounded_end_pose_problem.def_readonly("cost", &BoundedEndPoseProblem::cost);

    py::class_<SamplingProblem, std::shared_ptr<SamplingProblem>, PlanningProblem> sampling_problem(prob, "SamplingProblem");
    sampling_problem.def("update", &SamplingProblem::Update);
    sampling_problem.def_property("goal_state", &SamplingProblem::GetGoalState, &SamplingProblem::SetGoalState);
    sampling_problem.def("get_space_dim", &SamplingProblem::GetSpaceDim);
    sampling_problem.def("get_bounds", &SamplingProblem::GetBounds);
    sampling_problem.def_readonly("num_tasks", &SamplingProblem::num_tasks);
    sampling_problem.def_readonly("Phi", &SamplingProblem::Phi);
    sampling_problem.def_readonly("inequality", &SamplingProblem::inequality);
    sampling_problem.def_readonly("equality", &SamplingProblem::equality);
    sampling_problem.def("set_goal_eq", &SamplingProblem::SetGoalEQ);
    sampling_problem.def("set_rho_eq", &SamplingProblem::SetRhoEQ);
    sampling_problem.def("get_goal_eq", &SamplingProblem::GetGoalEQ);
    sampling_problem.def("get_rho_eq", &SamplingProblem::GetRhoEQ);
    sampling_problem.def("set_goal_neq", &SamplingProblem::SetGoalNEQ);
    sampling_problem.def("set_rho_neq", &SamplingProblem::SetRhoNEQ);
    sampling_problem.def("get_goal_neq", &SamplingProblem::GetGoalNEQ);
    sampling_problem.def("get_rho_neq", &SamplingProblem::GetRhoNEQ);
    sampling_problem.def("is_state_valid", &SamplingProblem::IsStateValid);

    py::class_<TimeIndexedSamplingProblem, std::shared_ptr<TimeIndexedSamplingProblem>, PlanningProblem> time_indexed_sampling_problem(prob, "TimeIndexedSamplingProblem");
    time_indexed_sampling_problem.def("update", &TimeIndexedSamplingProblem::Update);
    time_indexed_sampling_problem.def("get_space_dim", &TimeIndexedSamplingProblem::GetSpaceDim);
    time_indexed_sampling_problem.def("get_bounds", &TimeIndexedSamplingProblem::GetBounds);
    time_indexed_sampling_problem.def_property("goal_state", &TimeIndexedSamplingProblem::GetGoalState, &TimeIndexedSamplingProblem::SetGoalState);
    time_indexed_sampling_problem.def_property("goal_time", &TimeIndexedSamplingProblem::GetGoalTime, &TimeIndexedSamplingProblem::SetGoalTime);
    time_indexed_sampling_problem.def_readonly("num_tasks", &TimeIndexedSamplingProblem::num_tasks);
    time_indexed_sampling_problem.def_readonly("Phi", &TimeIndexedSamplingProblem::Phi);
    time_indexed_sampling_problem.def_readonly("inequality", &TimeIndexedSamplingProblem::inequality);
    time_indexed_sampling_problem.def_readonly("equality", &TimeIndexedSamplingProblem::equality);
    time_indexed_sampling_problem.def("set_goal_eq", &TimeIndexedSamplingProblem::SetGoalEQ);
    time_indexed_sampling_problem.def("set_rho_eq", &TimeIndexedSamplingProblem::SetRhoEQ);
    time_indexed_sampling_problem.def("get_goal_eq", &TimeIndexedSamplingProblem::GetGoalEQ);
    time_indexed_sampling_problem.def("get_rho_eq", &TimeIndexedSamplingProblem::GetRhoEQ);
    time_indexed_sampling_problem.def("set_goal_neq", &TimeIndexedSamplingProblem::SetGoalNEQ);
    time_indexed_sampling_problem.def("set_rho_neq", &TimeIndexedSamplingProblem::SetRhoNEQ);
    time_indexed_sampling_problem.def("get_goal_neq", &TimeIndexedSamplingProblem::GetGoalNEQ);
    time_indexed_sampling_problem.def("get_rho_neq", &TimeIndexedSamplingProblem::GetRhoNEQ);
    time_indexed_sampling_problem.def("is_valid", (bool (TimeIndexedSamplingProblem::*)(Eigen::VectorXdRefConst, const double&)) & TimeIndexedSamplingProblem::IsValid);

    py::enum_<ControlCostLossTermType>(module, "ControlCostLossTermType")
        // BimodalHuber = 3, SuperHuber = 4, <-- skipped as not actively used right now.
        .value("Undefined", ControlCostLossTermType::Undefined)
        .value("L2", ControlCostLossTermType::L2)
        .value("SmoothL1", ControlCostLossTermType::SmoothL1)
        .value("Huber", ControlCostLossTermType::Huber)
        .value("PseudoHuber", ControlCostLossTermType::PseudoHuber)
        .export_values();

    py::class_<DynamicTimeIndexedShootingProblem, std::shared_ptr<DynamicTimeIndexedShootingProblem>, PlanningProblem>(prob, "DynamicTimeIndexedShootingProblem")
        .def("update", (void (DynamicTimeIndexedShootingProblem::*)(Eigen::VectorXdRefConst, Eigen::VectorXdRefConst, int)) & DynamicTimeIndexedShootingProblem::Update)
        .def("update", (void (DynamicTimeIndexedShootingProblem::*)(Eigen::VectorXdRefConst, int)) & DynamicTimeIndexedShootingProblem::Update)
        .def("update_terminal_state", &DynamicTimeIndexedShootingProblem::UpdateTerminalState)
        .def_property("X", static_cast<const Eigen::MatrixXd& (DynamicTimeIndexedShootingProblem::*)(void)const>(&DynamicTimeIndexedShootingProblem::get_X), &DynamicTimeIndexedShootingProblem::set_X)
        .def_property("U", static_cast<const Eigen::MatrixXd& (DynamicTimeIndexedShootingProblem::*)(void)const>(&DynamicTimeIndexedShootingProblem::get_U), &DynamicTimeIndexedShootingProblem::set_U)
        .def_property("X_star", &DynamicTimeIndexedShootingProblem::get_X_star, &DynamicTimeIndexedShootingProblem::set_X_star)
        .def_property_readonly("tau", &DynamicTimeIndexedShootingProblem::get_tau)
        .def_property("T", &DynamicTimeIndexedShootingProblem::get_T, &DynamicTimeIndexedShootingProblem::set_T)
        .def_property("loss_type", &DynamicTimeIndexedShootingProblem::get_loss_type, &DynamicTimeIndexedShootingProblem::set_loss_type)
        .def_property("control_cost_weight", &DynamicTimeIndexedShootingProblem::get_control_cost_weight, &DynamicTimeIndexedShootingProblem::set_control_cost_weight)
        .def("get_Q", &DynamicTimeIndexedShootingProblem::get_Q)
        .def("set_Q", &DynamicTimeIndexedShootingProblem::set_Q)
        .def_readonly("Phi", &DynamicTimeIndexedShootingProblem::Phi)
        .def_readonly("dPhi_dx", &DynamicTimeIndexedShootingProblem::dPhi_dx)
        .def_readonly("cost", &DynamicTimeIndexedShootingProblem::cost)
        .def("get_state_cost", &DynamicTimeIndexedShootingProblem::GetStateCost)
        .def("get_state_cost_jacobian", &DynamicTimeIndexedShootingProblem::GetStateCostJacobian)
        .def("get_state_cost_hessian", &DynamicTimeIndexedShootingProblem::GetStateCostHessian)
        .def("get_control_cost", &DynamicTimeIndexedShootingProblem::GetControlCost)
        .def("get_control_cost_jacobian", &DynamicTimeIndexedShootingProblem::GetControlCostJacobian)
        .def("get_control_cost_hessian", &DynamicTimeIndexedShootingProblem::GetControlCostHessian)
        .def("get_state_cost_hessian", &DynamicTimeIndexedShootingProblem::GetStateControlCostHessian);

    py::class_<CollisionProxy, std::shared_ptr<CollisionProxy>> collision_proxy(module, "CollisionProxy");
    collision_proxy.def(py::init());
    collision_proxy.def_readonly("contact_1", &CollisionProxy::contact1);
    collision_proxy.def_readonly("contact_2", &CollisionProxy::contact2);
    collision_proxy.def_readonly("normal_1", &CollisionProxy::normal1);
    collision_proxy.def_readonly("normal_2", &CollisionProxy::normal2);
    collision_proxy.def_readonly("distance", &CollisionProxy::distance);
    collision_proxy.def_property_readonly("object_1", [](CollisionProxy* instance) { return (instance->e1 && instance->e2) ? instance->e1->segment.getName() : std::string(""); });
    collision_proxy.def_property_readonly("object_2", [](CollisionProxy* instance) { return (instance->e1 && instance->e2) ? instance->e2->segment.getName() : std::string(""); });
    collision_proxy.def_property_readonly("transform_1", [](CollisionProxy* instance) { return (instance->e1 && instance->e2) ? instance->e1->frame : KDL::Frame(); });
    collision_proxy.def_property_readonly("transform_2", [](CollisionProxy* instance) { return (instance->e1 && instance->e2) ? instance->e2->frame : KDL::Frame(); });
    collision_proxy.def("__repr__", &CollisionProxy::Print);

    py::class_<ContinuousCollisionProxy, std::shared_ptr<ContinuousCollisionProxy>> continuous_collision_proxy(module, "ContinuousCollisionProxy");
    continuous_collision_proxy.def(py::init());
    continuous_collision_proxy.def_readonly("contact_transform_1", &ContinuousCollisionProxy::contact_tf1);
    continuous_collision_proxy.def_readonly("contact_transform_2", &ContinuousCollisionProxy::contact_tf2);
    continuous_collision_proxy.def_readonly("in_collision", &ContinuousCollisionProxy::in_collision);
    continuous_collision_proxy.def_readonly("time_of_contact", &ContinuousCollisionProxy::time_of_contact);
    continuous_collision_proxy.def_property_readonly("object_1", [](ContinuousCollisionProxy* instance) { return (instance->e1 && instance->e2) ? instance->e1->segment.getName() : std::string(""); });
    continuous_collision_proxy.def_property_readonly("object_2", [](ContinuousCollisionProxy* instance) { return (instance->e1 && instance->e2) ? instance->e2->segment.getName() : std::string(""); });
    continuous_collision_proxy.def_property_readonly("transform_1", [](ContinuousCollisionProxy* instance) { return (instance->e1 && instance->e2) ? instance->e1->frame : KDL::Frame(); });
    continuous_collision_proxy.def_property_readonly("transform_2", [](ContinuousCollisionProxy* instance) { return (instance->e1 && instance->e2) ? instance->e2->frame : KDL::Frame(); });
    continuous_collision_proxy.def("__repr__", &ContinuousCollisionProxy::Print);

    py::class_<Scene, std::shared_ptr<Scene>, Object> scene(module, "Scene");
    scene.def_property_readonly("num_positions", &Scene::get_num_positions);
    scene.def_property_readonly("num_velocities", &Scene::get_num_velocities);
    scene.def_property_readonly("num_controls", &Scene::get_num_controls);
    scene.def_property_readonly("num_state", &Scene::get_num_state);
    scene.def_property_readonly("num_state_derivative", &Scene::get_num_state_derivative);
    scene.def_property_readonly("has_quaternion_floating_base", &Scene::get_has_quaternion_floating_base);
    scene.def("update", &Scene::Update, py::arg("x"), py::arg("t") = 0.0);
    scene.def("get_controlled_joint_names", (std::vector<std::string>(Scene::*)()) & Scene::GetControlledJointNames);
    scene.def("get_controlled_link_names", &Scene::GetControlledLinkNames);
    scene.def("get_model_link_names", &Scene::GetModelLinkNames);
    scene.def("get_kinematic_tree", &Scene::GetKinematicTree, py::return_value_policy::reference_internal);
    scene.def("get_collision_scene", &Scene::GetCollisionScene, py::return_value_policy::reference_internal);
    scene.def("get_dynamics_solver", &Scene::GetDynamicsSolver, py::return_value_policy::reference_internal);
    scene.def("get_model_joint_names", &Scene::GetModelJointNames);
    scene.def("get_model_state", &Scene::GetModelState);
    scene.def("get_model_state_map", &Scene::GetModelStateMap);
    scene.def("get_tree_names", [](Scene& scene) {
        std::vector<std::string> frame_names;
        for (const auto& m : scene.GetTreeMap())
        {
            frame_names.push_back(m.first);
        }
        return frame_names;
    });
    scene.def("set_model_state", (void (Scene::*)(Eigen::VectorXdRefConst, double, bool)) & Scene::SetModelState, py::arg("x"), py::arg("t") = 0.0, py::arg("update_trajectory") = false);
    scene.def("set_model_state_map", (void (Scene::*)(const std::map<std::string, double>&, double, bool)) & Scene::SetModelState, py::arg("x"), py::arg("t") = 0.0, py::arg("update_trajectory") = false);
    scene.def("get_controlled_state", &Scene::GetControlledState);
    scene.def("publish_scene", &Scene::PublishScene);
    scene.def("publish_proxies", &Scene::PublishProxies);
    scene.def("update_planning_scene", &Scene::UpdatePlanningScene);
    scene.def("load_scene",
              (void (Scene::*)(const std::string&, const KDL::Frame&, bool)) & Scene::LoadScene,
              py::arg("scene_string"),
              py::arg("offset_transform") = kdl_frame(),
              py::arg("update_collision_scene") = true);
    scene.def("load_scene_file",
              (void (Scene::*)(const std::string&, const KDL::Frame&, bool)) & Scene::LoadSceneFile,
              py::arg("file_name"),
              py::arg("offset_transform") = kdl_frame(),
              py::arg("update_collision_scene") = true);
    scene.def("get_scene", &Scene::GetScene);
    scene.def("clean_scene", &Scene::CleanScene);
    scene.def("is_state_valid", [](Scene* instance, bool self, double safe_distance) { return instance->GetCollisionScene()->IsStateValid(self, safe_distance); }, py::arg("check_self_collision") = true, py::arg("safe_distance") = 0.0);
    scene.def("is_collision_free", [](Scene* instance, const std::string& o1, const std::string& o2, double safe_distance) { return instance->GetCollisionScene()->IsCollisionFree(o1, o2, safe_distance); }, py::arg("object_1"), py::arg("object_2"), py::arg("safe_distance") = 0.0);
    scene.def("is_allowed_to_collide", [](Scene* instance, const std::string& o1, const std::string& o2, bool self) { return instance->GetCollisionScene()->IsAllowedToCollide(o1, o2, self); }, py::arg("object_1"), py::arg("object_2"), py::arg("check_self_collision") = true);
    scene.def("get_collision_distance", [](Scene* instance, bool self) { return instance->GetCollisionScene()->GetCollisionDistance(self); }, py::arg("check_self_collision") = true);
    scene.def("get_collision_distance", [](Scene* instance, const std::string& o1, const std::string& o2) { return instance->GetCollisionScene()->GetCollisionDistance(o1, o2); }, py::arg("object_1"), py::arg("object_2"));
    scene.def("get_collision_distance",
              [](Scene* instance, const std::string& o1, const bool& self) {
                  return instance->GetCollisionScene()->GetCollisionDistance(o1, self);
              },
              py::arg("object_1"), py::arg("check_self_collision") = true);
    scene.def("get_collision_distance",
              [](Scene* instance, const std::vector<std::string>& objects, const bool& self) {
                  return instance->GetCollisionScene()->GetCollisionDistance(objects, self);
              },
              py::arg("objects"), py::arg("check_self_collision") = true);
    scene.def("update_planning_scene_world",
              [](Scene* instance, moveit_msgs::PlanningSceneWorld& world) {
                  moveit_msgs::PlanningSceneWorldConstPtr my_ptr(
                      new moveit_msgs::PlanningSceneWorld(world));
                  instance->UpdatePlanningSceneWorld(my_ptr);
              });
    scene.def("update_collision_objects", &Scene::UpdateCollisionObjects);
    scene.def("get_collision_robot_links", [](Scene* instance) { return instance->GetCollisionScene()->GetCollisionRobotLinks(); });
    scene.def("get_collision_world_links", [](Scene* instance) { return instance->GetCollisionScene()->GetCollisionWorldLinks(); });
    scene.def("get_root_frame_name", &Scene::GetRootFrameName);
    scene.def("get_root_joint_name", &Scene::GetRootJointName);
    scene.def("attach_object", &Scene::AttachObject);
    scene.def("attach_object_local", (void (Scene::*)(const std::string& name, const std::string& parent, const KDL::Frame& pose)) & Scene::AttachObjectLocal);
    scene.def("attach_object_local", (void (Scene::*)(const std::string& name, const std::string& parent, const Eigen::VectorXd& pose)) & Scene::AttachObjectLocal);
    scene.def("detach_object", &Scene::DetachObject);
    scene.def("has_attached_object", &Scene::HasAttachedObject);
    scene.def("fk", [](Scene* instance, const std::string& e1, const KDL::Frame& o1, const std::string& e2, const KDL::Frame& o2) { return instance->GetKinematicTree().FK(e1, o1, e2, o2); });
    scene.def("fk", [](Scene* instance, const std::string& e1, const std::string& e2) { return instance->GetKinematicTree().FK(e1, KDL::Frame(), e2, KDL::Frame()); });
    scene.def("fk", [](Scene* instance, const std::string& e1) { return instance->GetKinematicTree().FK(e1, KDL::Frame(), "", KDL::Frame()); });
    scene.def("jacobian", [](Scene* instance, const std::string& e1, const KDL::Frame& o1, const std::string& e2, const KDL::Frame& o2) { return instance->GetKinematicTree().Jacobian(e1, o1, e2, o2); });
    scene.def("jacobian", [](Scene* instance, const std::string& e1, const std::string& e2) { return instance->GetKinematicTree().Jacobian(e1, KDL::Frame(), e2, KDL::Frame()); });
    scene.def("jacobian", [](Scene* instance, const std::string& e1) { return instance->GetKinematicTree().Jacobian(e1, KDL::Frame(), "", KDL::Frame()); });
    scene.def("hessian", [](Scene* instance, const std::string& e1, const KDL::Frame& o1, const std::string& e2, const KDL::Frame& o2) { return instance->GetKinematicTree().Hessian(e1, o1, e2, o2); });
    scene.def("hessian", [](Scene* instance, const std::string& e1, const std::string& e2) { return instance->GetKinematicTree().Hessian(e1, KDL::Frame(), e2, KDL::Frame()); });
    scene.def("hessian", [](Scene* instance, const std::string& e1) { return instance->GetKinematicTree().Hessian(e1, KDL::Frame(), "", KDL::Frame()); });
    scene.def("add_trajectory_from_file", &Scene::AddTrajectoryFromFile);
    scene.def("add_trajectory", (void (Scene::*)(const std::string&, const std::string&)) & Scene::AddTrajectory);
    scene.def("add_trajectory_from_array",
              [](Scene* instance, const std::string& link, const Eigen::MatrixXd& data, double radius) {
                  instance->AddTrajectory(link, std::shared_ptr<Trajectory>(new Trajectory(data, radius)));
              },
              py::arg("link"), py::arg("data"), py::arg("radius"));
    scene.def("get_trajectory", [](Scene* instance, const std::string& link) { return instance->GetTrajectory(link)->ToString(); });
    scene.def("remove_trajectory", &Scene::RemoveTrajectory);
    scene.def("update_scene_frames", &Scene::UpdateSceneFrames);
    scene.def("add_object", [](Scene* instance, const std::string& name, const KDL::Frame& transform, const std::string& parent, const std::string& shape_resource_path, const Eigen::Vector3d scale, const Eigen::Vector4d color, const bool update_collision_scene) { instance->AddObject(name, transform, parent, shape_resource_path, scale, KDL::RigidBodyInertia::Zero(), color, update_collision_scene); },
              py::arg("name"),
              py::arg("transform") = KDL::Frame(),
              py::arg("parent") = "",
              py::arg("shape_resource_path"),
              py::arg("scale") = Eigen::Vector3d::Ones(),
              py::arg("color") = Eigen::Vector4d(0.5, 0.5, 0.5, 1.0),
              py::arg("update_collision_scene") = true);
    scene.def("add_object", (void (Scene::*)(const std::string&, const KDL::Frame&, const std::string&, shapes::ShapeConstPtr, const KDL::RigidBodyInertia&, const Eigen::Vector4d&, const bool)) & Scene::AddObject,
              py::arg("name"),
              py::arg("transform") = KDL::Frame(),
              py::arg("parent") = std::string(),
              py::arg("shape"),
              py::arg("inertia") = KDL::RigidBodyInertia::Zero(),
              py::arg("color") = Eigen::Vector4d(0.5, 0.5, 0.5, 1.0),
              py::arg("update_collision_scene") = true);
    scene.def("add_object_to_environment", &Scene::AddObjectToEnvironment,
              py::arg("name"),
              py::arg("transform") = KDL::Frame(),
              py::arg("shape"),
              py::arg("color") = Eigen::Vector4d(0.5, 0.5, 0.5, 1.0),
              py::arg("update_collision_scene") = true);
    scene.def("remove_object", &Scene::RemoveObject);
    scene.def_property_readonly("model_link_to_collision_link_map", &Scene::GetModelLinkToCollisionLinkMap);
    scene.def_property_readonly("controlled_joint_to_collision_link_map", &Scene::GetControlledJointToCollisionLinkMap);
    scene.def_property_readonly("world_links_to_exclude_from_collision_scene", &Scene::get_world_links_to_exclude_from_collision_scene);

    py::class_<CollisionScene, std::shared_ptr<CollisionScene>> collision_scene(module, "CollisionScene");
    // TODO: expose IsStateValid, IsCollisionFree, GetCollisionDistance, GetCollisionWorldLinks, GetCollisionRobotLinks, GetTranslation
    collision_scene.def_property("always_externally_updated_collision_scene", &CollisionScene::GetAlwaysExternallyUpdatedCollisionScene, &CollisionScene::SetAlwaysExternallyUpdatedCollisionScene);
    collision_scene.def_property("replace_primitive_shapes_with_meshes", &CollisionScene::GetReplacePrimitiveShapesWithMeshes, &CollisionScene::SetReplacePrimitiveShapesWithMeshes);
    collision_scene.def_property("replace_cylinders_with_capsules", &CollisionScene::get_replace_cylinders_with_capsules, &CollisionScene::set_replace_cylinders_with_capsules);
    collision_scene.def_property("robot_link_scale", &CollisionScene::GetRobotLinkScale, &CollisionScene::SetRobotLinkScale);
    collision_scene.def_property("world_link_scale", &CollisionScene::GetWorldLinkScale, &CollisionScene::SetWorldLinkScale);
    collision_scene.def_property("robot_link_padding", &CollisionScene::GetRobotLinkPadding, &CollisionScene::SetRobotLinkPadding);
    collision_scene.def_property("world_link_padding", &CollisionScene::GetWorldLinkPadding, &CollisionScene::SetWorldLinkPadding);
    collision_scene.def("update_collision_object_transforms", &CollisionScene::UpdateCollisionObjectTransforms);
    collision_scene.def("continuous_collision_check", &CollisionScene::ContinuousCollisionCheck);
    collision_scene.def("get_robot_to_robot_collision_distance", &CollisionScene::GetRobotToRobotCollisionDistance);
    collision_scene.def("get_robot_to_world_collision_distance", &CollisionScene::GetRobotToWorldCollisionDistance);
    collision_scene.def("get_translation", &CollisionScene::GetTranslation);

    py::class_<VisualizationMoveIt> visualization_moveit(module, "VisualizationMoveIt");
    visualization_moveit.def(py::init<ScenePtr>());
    visualization_moveit.def("display_trajectory", &VisualizationMoveIt::DisplayTrajectory);
#ifdef MSGPACK_FOUND
    py::class_<VisualizationMeshcat> visualization_meshcat(module, "VisualizationMeshcat");
    visualization_meshcat.def(py::init<ScenePtr, const std::string&, bool, const std::string&>(), py::arg("scene"), py::arg("url"), py::arg("use_mesh_materials") = true, py::arg("file_url") = "");
    visualization_meshcat.def("display_scene", &VisualizationMeshcat::DisplayScene, py::arg("use_mesh_materials") = true);
    visualization_meshcat.def("display_state", &VisualizationMeshcat::DisplayState, py::arg("state"), py::arg("t") = 0.0);
    visualization_meshcat.def("display_trajectory", &VisualizationMeshcat::DisplayTrajectory, py::arg("trajectory"), py::arg("dt") = 1.0);
    visualization_meshcat.def("get_web_url", &VisualizationMeshcat::GetWebURL);
    visualization_meshcat.def("get_file_url", &VisualizationMeshcat::GetFileURL);
    visualization_meshcat.def("delete", &VisualizationMeshcat::Delete, py::arg("path") = "");
    visualization_meshcat.def("set_property", py::overload_cast<const std::string&, const std::string&, const double&>(&VisualizationMeshcat::SetProperty), py::arg("path"), py::arg("property"), py::arg("value"));
    visualization_meshcat.def("set_property", py::overload_cast<const std::string&, const std::string&, const std::string&>(&VisualizationMeshcat::SetProperty), py::arg("path"), py::arg("property"), py::arg("value"));
    visualization_meshcat.def("set_property", py::overload_cast<const std::string&, const std::string&, const bool&>(&VisualizationMeshcat::SetProperty), py::arg("path"), py::arg("property"), py::arg("value"));
    visualization_meshcat.def("set_property", py::overload_cast<const std::string&, const std::string&, const Eigen::Vector3d&>(&VisualizationMeshcat::SetProperty), py::arg("path"), py::arg("property"), py::arg("value"));
    visualization_meshcat.def("set_property", py::overload_cast<const std::string&, const std::string&, const Eigen::Vector4d&>(&VisualizationMeshcat::SetProperty), py::arg("path"), py::arg("property"), py::arg("value"));
#endif

    py::module kin = module.def_submodule("Kinematics", "Kinematics submodule.");
    py::class_<KinematicTree, std::shared_ptr<KinematicTree>> kinematic_tree(kin, "KinematicTree");
    kinematic_tree.def_readwrite("debug_mode", &KinematicTree::debug);
    kinematic_tree.def("publish_frames", &KinematicTree::PublishFrames, py::arg("tf_prefix") = "exotica");
    kinematic_tree.def("get_root_frame_name", &KinematicTree::GetRootFrameName);
    kinematic_tree.def("get_root_joint_name", &KinematicTree::GetRootJointName);
    kinematic_tree.def("get_kinematic_chain", &KinematicTree::GetKinematicChain);
    kinematic_tree.def("get_kinematic_chain_links", &KinematicTree::GetKinematicChainLinks);
    kinematic_tree.def("get_model_base_type", &KinematicTree::GetModelBaseType);
    kinematic_tree.def("get_controlled_base_type", &KinematicTree::GetControlledBaseType);
    kinematic_tree.def("get_controlled_link_mass", &KinematicTree::GetControlledLinkMass);
    kinematic_tree.def("get_collision_object_types", &KinematicTree::GetCollisionObjectTypes);
    kinematic_tree.def("set_seed", &KinematicTree::SetSeed);
    kinematic_tree.def("get_random_controlled_state", &KinematicTree::GetRandomControlledState);
    kinematic_tree.def("get_num_model_joints", &KinematicTree::GetNumModelJoints);
    kinematic_tree.def("get_num_controlled_joints", &KinematicTree::GetNumControlledJoints);
    kinematic_tree.def("find_kinematic_element_by_name", &KinematicTree::FindKinematicElementByName);

    // joints and links that describe the full state of the robot
    kinematic_tree.def("get_model_link_names", &KinematicTree::GetModelLinkNames);
    kinematic_tree.def("get_model_joint_names", &KinematicTree::GetModelJointNames);

    // subset of model joints and links that can be controlled
    kinematic_tree.def("get_controlled_link_names", &KinematicTree::GetControlledLinkNames);
    kinematic_tree.def("get_controlled_joint_names", &KinematicTree::GetControlledJointNames);

    // Joint Limits
    kinematic_tree.def("get_joint_limits", &KinematicTree::GetJointLimits);
    kinematic_tree.def("reset_joint_limits", &KinematicTree::ResetJointLimits);
    kinematic_tree.def("set_joint_limits_lower", &KinematicTree::SetJointLimitsLower);
    kinematic_tree.def("set_joint_limits_upper", &KinematicTree::SetJointLimitsUpper);
    kinematic_tree.def("set_joint_velocity_limits", &KinematicTree::SetJointVelocityLimits);
    kinematic_tree.def("set_joint_acceleration_limits", &KinematicTree::SetJointAccelerationLimits);
    kinematic_tree.def("get_velocity_limits", &KinematicTree::GetVelocityLimits);
    kinematic_tree.def("has_acceleration_limits", &KinematicTree::HasAccelerationLimits);
    kinematic_tree.def("get_acceleration_limits", &KinematicTree::GetAccelerationLimits);
    kinematic_tree.def("set_floating_base_limits_pos_xyz_euler_zyx", (void (KinematicTree::*)(const std::vector<double>&, const std::vector<double>&)) & KinematicTree::SetFloatingBaseLimitsPosXYZEulerZYX);
    kinematic_tree.def("set_floating_base_limits_pos_xyz_euler_zyx", (void (KinematicTree::*)(const std::vector<double>&, const std::vector<double>&, const std::vector<double>&, const std::vector<double>&)) & KinematicTree::SetFloatingBaseLimitsPosXYZEulerZYX);
    kinematic_tree.def("set_planar_base_limits_pos_xy_euler_z", (void (KinematicTree::*)(const std::vector<double>&, const std::vector<double>&)) & KinematicTree::SetPlanarBaseLimitsPosXYEulerZ);
    kinematic_tree.def("set_planar_base_limits_pos_xy_euler_z", (void (KinematicTree::*)(const std::vector<double>&, const std::vector<double>&, const std::vector<double>&, const std::vector<double>&)) & KinematicTree::SetPlanarBaseLimitsPosXYEulerZ);
    kinematic_tree.def("get_used_joint_limits", &KinematicTree::GetUsedJointLimits);

    // Get full tree
    kinematic_tree.def("get_model_tree", &KinematicTree::GetModelTree);
    kinematic_tree.def("get_tree", [](KinematicTree* kt) {
        auto tree_weak_ptr = kt->GetTree();
        std::vector<std::shared_ptr<KinematicElement>> tree_shared_ptr;
        tree_shared_ptr.reserve(tree_weak_ptr.size());
        for (auto e : tree_weak_ptr)
            tree_shared_ptr.emplace_back(e.lock());
        return tree_shared_ptr;
    });

    // KinematicElement
    py::class_<KinematicElement, std::shared_ptr<KinematicElement>> kinematic_element(kin, "KinematicElement");
    kinematic_element.def("get_pose", &KinematicElement::GetPose);
    kinematic_element.def_readonly("id", &KinematicElement::id);
    kinematic_element.def("get_segment_name", [](KinematicElement* element) { return element->segment.getName(); });
    kinematic_element.def("get_joint_name", [](KinematicElement* element) { return element->segment.getJoint().getName(); });
    kinematic_element.def("get_parent_name", [](KinematicElement* element) { auto parent = element->parent.lock(); if (parent) { return parent->segment.getName(); } else { return std::string("no_parent"); } });
    kinematic_element.def("get_mass", [](KinematicElement* element) { return element->segment.getInertia().getMass(); });
    kinematic_element.def_readonly("control_id", &KinematicElement::control_id);
    kinematic_element.def_readonly("is_controlled", &KinematicElement::is_controlled);
    kinematic_element.def_readonly("parent_name", &KinematicElement::parent_name);
    kinematic_element.def_readonly("joint_limits", &KinematicElement::joint_limits);
    kinematic_element.def_readonly("is_robot_link", &KinematicElement::is_robot_link);
    kinematic_element.def_readonly("shape", &KinematicElement::shape);

    // TODO: KinematicRequestFlags

    // TODO: KinematicFrame

    // KinematicResponse
    py::class_<KinematicResponse, std::shared_ptr<KinematicResponse>> kinematic_response(kin, "KinematicResponse");
    kinematic_response.def_property_readonly("Phi", [](KinematicResponse* instance) {
        std::vector<KDL::Frame> vec;
        for (unsigned int i = 0; i < instance->Phi.cols(); ++i)
            vec.push_back(instance->Phi(i));
        return vec;
    });

    py::enum_<Integrator>(module, "Integrator")
        .value("RK1", Integrator::RK1)
        .value("SymplecticEuler", Integrator::SymplecticEuler)
        .value("RK2", Integrator::RK2)
        .value("RK4", Integrator::RK4)
        .export_values();

    py::class_<DynamicsSolver, std::shared_ptr<DynamicsSolver>, Object>(module, "DynamicsSolver")
        .def("F", &DynamicsSolver::F)
        .def("f", &DynamicsSolver::f)
        .def("fx", &DynamicsSolver::fx)
        .def("fu", &DynamicsSolver::fu)
        .def("fx_fd", &DynamicsSolver::fx_fd)
        .def("fu_fd", &DynamicsSolver::fu_fd)
        .def_property_readonly("nq", &DynamicsSolver::get_num_positions)
        .def_property_readonly("nv", &DynamicsSolver::get_num_velocities)
        .def_property_readonly("nx", &DynamicsSolver::get_num_state)
        .def_property_readonly("ndx", &DynamicsSolver::get_num_state_derivative)
        .def_property_readonly("nu", &DynamicsSolver::get_num_controls)
        .def_property_readonly("has_second_order_derivatives", &DynamicsSolver::get_has_second_order_derivatives)
        .def_property("integrator", &DynamicsSolver::get_integrator, &DynamicsSolver::set_integrator)
        .def("get_position", &DynamicsSolver::GetPosition)
        .def("simulate", &DynamicsSolver::Simulate)
        .def("state_delta", (Eigen::VectorXd(DynamicsSolver::*)(const Eigen::VectorXd&, const Eigen::VectorXd&)) & DynamicsSolver::StateDelta)
        .def("state_delta_derivative", &DynamicsSolver::dStateDelta)
        .def("state_delta_second_derivative", &DynamicsSolver::ddStateDelta)
        .def("compute_derivatives", &DynamicsSolver::ComputeDerivatives)
        .def("get_Fx", &DynamicsSolver::get_Fx)
        .def("get_Fu", &DynamicsSolver::get_Fu)
        .def("get_fx", &DynamicsSolver::get_fx)
        .def("get_fu", &DynamicsSolver::get_fu)
        .def("integrate", [](DynamicsSolver* instance, Eigen::VectorXdRefConst x, Eigen::VectorXdRefConst u, const double dt) {
            Eigen::VectorXd xout(instance->get_num_positions() + instance->get_num_velocities());
            instance->Integrate(x, u, dt, xout);
            return xout;
        })
        .def_property_readonly("dt", &DynamicsSolver::get_dt, "dt");

    ////////////////////////////////////////////////////////////////////////////
    /// Shapes

    // shape base class
    py::class_<shapes::Shape, shapes::ShapePtr>(module, "Shape")
        .def("scale", &shapes::Shape::scale)
        .def("padd", &shapes::Shape::padd)
        .def("scaleAndPadd", &shapes::Shape::scaleAndPadd)
        .def("isFixed", &shapes::Shape::isFixed)
        .def_readonly("type", &shapes::Shape::type);

    py::class_<shapes::Sphere, shapes::Shape, std::shared_ptr<shapes::Sphere>>(module, "Sphere")
        .def(py::init())
        .def(py::init<double>())
        .def_readonly_static("name", &shapes::Sphere::STRING_NAME)
        .def_readwrite("radius", &shapes::Sphere::radius);

    py::class_<shapes::Cylinder, shapes::Shape, std::shared_ptr<shapes::Cylinder>>(module, "Cylinder")
        .def(py::init())
        .def(py::init<double, double>())
        .def_readonly_static("name", &shapes::Cylinder::STRING_NAME)
        .def_readwrite("radius", &shapes::Cylinder::radius)
        .def_readwrite("length", &shapes::Cylinder::length);

    py::class_<shapes::Cone, shapes::Shape, std::shared_ptr<shapes::Cone>>(module, "Cone")
        .def(py::init())
        .def(py::init<double, double>())
        .def_readonly_static("name", &shapes::Cone::STRING_NAME)
        .def_readwrite("radius", &shapes::Cone::radius)
        .def_readwrite("length", &shapes::Cone::length);

    py::class_<shapes::Box, shapes::Shape, std::shared_ptr<shapes::Box>>(module, "Box")
        .def(py::init())
        .def(py::init<double, double, double>())
        .def_readonly_static("name", &shapes::Box::STRING_NAME);

    py::class_<shapes::Plane, shapes::Shape, std::shared_ptr<shapes::Plane>>(module, "Plane")
        .def(py::init())
        .def(py::init<double, double, double, double>())
        .def_readonly_static("name", &shapes::Plane::STRING_NAME)
        .def("isFixed", &shapes::Plane::isFixed)
        .def_readwrite("a", &shapes::Plane::a)
        .def_readwrite("b", &shapes::Plane::b)
        .def_readwrite("c", &shapes::Plane::c)
        .def_readwrite("d", &shapes::Plane::d);

    py::class_<shapes::Mesh, shapes::Shape, std::shared_ptr<shapes::Mesh>>(module, "Mesh")
        .def(py::init())
        .def(py::init<unsigned int, unsigned int>())
        .def("computeTriangleNormals", &shapes::Mesh::computeTriangleNormals)
        .def("computeVertexNormals", &shapes::Mesh::computeVertexNormals)
        .def("mergeVertices", &shapes::Mesh::mergeVertices)
        .def_readonly("vertex_count", &shapes::Mesh::vertex_count)
        .def_readonly("triangle_count", &shapes::Mesh::triangle_count);

    py::class_<shapes::OcTree, shapes::Shape, std::shared_ptr<shapes::OcTree>>(module, "OcTree")
        .def(py::init())
        .def(py::init<const std::shared_ptr<const octomap::OcTree>&>());

    py::enum_<shapes::ShapeType>(module, "ShapeType")
        .value("UNKNOWN_SHAPE", shapes::ShapeType::UNKNOWN_SHAPE)
        .value("SPHERE", shapes::ShapeType::SPHERE)
        .value("CYLINDER", shapes::ShapeType::CYLINDER)
        .value("CONE", shapes::ShapeType::CONE)
        .value("BOX", shapes::ShapeType::BOX)
        .value("PLANE", shapes::ShapeType::PLANE)
        .value("MESH", shapes::ShapeType::MESH)
        .value("OCTREE", shapes::ShapeType::OCTREE)
        .export_values();

    py::enum_<ArgumentPosition>(module, "ArgumentPosition")
        .value("ARG0", ArgumentPosition::ARG0)
        .value("ARG1", ArgumentPosition::ARG1)
        .value("ARG2", ArgumentPosition::ARG2)
        .value("ARG3", ArgumentPosition::ARG3)
        .value("ARG4", ArgumentPosition::ARG4)
        .export_values();

    module.attr("version") = std::string(exotica::version);
    module.attr("branch") = std::string(exotica::branch);

    py::class_<BoxQPSolution>(module, "BoxQPSolution")
        .def_readonly("Hff_inv", &BoxQPSolution::Hff_inv)
        .def_readonly("x", &BoxQPSolution::x)
        .def_readonly("free_idx", &BoxQPSolution::free_idx)
        .def_readonly("clamped_idx", &BoxQPSolution::clamped_idx);

    module.def("box_qp",
               (BoxQPSolution(*)(const Eigen::MatrixXd& H, const Eigen::VectorXd& q,
                                 const Eigen::VectorXd& b_low, const Eigen::VectorXd& b_high,
                                 const Eigen::VectorXd& x_init, const double gamma,
                                 const int max_iterations, const double epsilon, const double lambda,
                                 bool use_polynomial_linesearch,
                                 bool use_cholesky_factorization)) &
                   BoxQP,
               py::arg("H"), py::arg("q"), py::arg("b_low"), py::arg("b_high"), py::arg("x_init"), py::arg("gamma"), py::arg("max_iterations"), py::arg("epsilon"), py::arg("lambda"), py::arg("use_polynomial_linesearch") = true, py::arg("use_cholesky_factorization") = true);

    module.def("box_qp_old",
               (BoxQPSolution(*)(const Eigen::MatrixXd& H, const Eigen::VectorXd& q,
                                 const Eigen::VectorXd& b_low, const Eigen::VectorXd& b_high,
                                 const Eigen::VectorXd& x_init, const double gamma,
                                 const int max_iterations, const double epsilon, const double lambda,
                                 bool use_polynomial_linesearch,
                                 bool use_cholesky_factorization)) &
                   ExoticaBoxQP,
               py::arg("H"), py::arg("q"), py::arg("b_low"), py::arg("b_high"), py::arg("x_init"), py::arg("gamma"), py::arg("max_iterations"), py::arg("epsilon"), py::arg("lambda"), py::arg("use_polynomial_linesearch") = false, py::arg("use_cholesky_factorization") = false);

    AddInitializers(module);

    auto cleanup_exotica = []() {
        known_initializers.clear();
        Setup::Destroy();
    };
    module.add_object("_cleanup", py::capsule(cleanup_exotica));
}
