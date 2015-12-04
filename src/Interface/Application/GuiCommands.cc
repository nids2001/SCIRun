/*
For more information, please see: http://software.sci.utah.edu

The MIT License

Copyright (c) 2015 Scientific Computing and Imaging Institute,
University of Utah.

License for the specific language governing rights and limitations under
Permission is hereby granted, free of charge, to any person obtaining a
copy of this software and associated documentation files (the "Software"),
to deal in the Software without restriction, including without limitation
the rights to use, copy, modify, merge, publish, distribute, sublicense,
and/or sell copies of the Software, and to permit persons to whom the
Software is furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included
in all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS
OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
DEALINGS IN THE SOFTWARE.
*/

#include <QtGui>
#include <numeric>
#include <Core/Application/Application.h>
#include <Core/Application/Preferences/Preferences.h>
#include <Interface/Application/SCIRunMainWindow.h>
#include <Interface/Application/GuiCommands.h>
#include <Interface/Application/GuiLogger.h>
#include <Interface/Application/NetworkEditor.h>
// ReSharper disable once CppUnusedIncludeDirective
#include <Interface/Application/NetworkEditorControllerGuiProxy.h>
#include <Dataflow/Serialization/Network/XMLSerializer.h>
#include <Dataflow/Serialization/Network/NetworkDescriptionSerialization.h>
#include <Dataflow/Serialization/Network/Importer/NetworkIO.h>
#include <Dataflow/Engine/Controller/NetworkEditorController.h>
#include <Interface/Application/Utility.h>
#include <Core/Logging/Log.h>
#include <boost/range/adaptors.hpp>
#include <boost/algorithm/string.hpp>

using namespace SCIRun::Gui;
using namespace SCIRun::Core;
using namespace Commands;
using namespace SCIRun::Dataflow::Networks;
using namespace Algorithms;

LoadFileCommandGui::LoadFileCommandGui()
{
  addParameter(Name("FileNum"), 0);
}

bool LoadFileCommandGui::execute()
{
  auto inputFiles = Application::Instance().parameters()->inputFiles();
  return SCIRunMainWindow::Instance()->loadNetworkFile(QString::fromStdString(inputFiles[index_]));
}

bool ExecuteCurrentNetworkCommandGui::execute()
{
  SCIRunMainWindow::Instance()->executeAll();
  return true;
}

bool QuitAfterExecuteCommandGui::execute()
{
  SCIRunMainWindow::Instance()->setupQuitAfterExecute();
  return true;
}

bool QuitCommandGui::execute()
{
  SCIRunMainWindow::Instance()->quit();
  exit(0);
  return true;
}

bool ShowMainWindowGui::execute()
{
  auto mainWin = SCIRunMainWindow::Instance();
  mainWin->activateWindow();

  mainWin->raise();
  mainWin->show();
  return true;
}

ShowSplashScreenGui::ShowSplashScreenGui()
{
  initSplashScreen();
}

bool ShowSplashScreenGui::execute()
{
  splash_->show();

  splashTimer_->start();
  splash_->showMessage("Welcome! Tip: Press F1 and click anywhere in the interface for helpful hints.", Qt::AlignBottom, Qt::white);
  qApp->processEvents();

  return true;
}

void ShowSplashScreenGui::initSplashScreen()
{
  splash_ = new QSplashScreen(0, QPixmap(":/general/Resources/scirun_5_0_alpha.png"), Qt::WindowStaysOnTopHint);
  splashTimer_ = new QTimer;
  splashTimer_->setSingleShot( true );
  splashTimer_->setInterval( 5000 );
  QObject::connect( splashTimer_, SIGNAL( timeout() ), splash_, SLOT( close() ));
}

QSplashScreen* ShowSplashScreenGui::splash_ = 0;
QTimer* ShowSplashScreenGui::splashTimer_ = 0;

namespace
{
  template <class PointIter>
  QPointF centroidOfPointRange(PointIter begin, PointIter end)
  {
    QPointF sum = std::accumulate(begin, end, QPointF(), [](const QPointF& acc, const typename PointIter::value_type& point) { return acc + QPointF(point.first, point.second); });
    size_t num = std::distance(begin, end);
    return sum / num;
  }

  QPointF findCenterOfNetworkFile(const NetworkFile& file)
  {
    return findCenterOfNetwork(file.modulePositions);
  }
}

QPointF SCIRun::Gui::findCenterOfNetwork(const ModulePositions& positions)
{
  auto pointRange = positions.modulePositions | boost::adaptors::map_values;
  return centroidOfPointRange(pointRange.begin(), pointRange.end());
}

namespace std
{
template <typename T1, typename T2>
std::ostream& operator<<(std::ostream& o, const std::pair<T1,T2>& p)
{
  return o << p.first << "," << p.second;
}
}

bool NetworkFileProcessCommand::execute()
{
  if (!filename_.empty())
    GuiLogger::Instance().logInfo("Attempting load of " + QString::fromStdString(filename_));

  try
  {
    auto file = processXmlFile();

    if (file)
    {
      auto load = boost::bind(&NetworkFileProcessCommand::guiProcess, this, file);
      if (Core::Application::Instance().parameters()->isRegressionMode())
      {
        load();
      }
      else
      {
        int numModules = static_cast<int>(file->network.modules.size());
        QProgressDialog progress("Loading network " + QString::fromStdString(filename_), QString(), 0, numModules + 1, SCIRunMainWindow::Instance());
        progress.connect(networkEditor_->getNetworkEditorController().get(), SIGNAL(networkDoneLoading(int)), SLOT(setValue(int)));
        progress.setWindowModality(Qt::WindowModal);
        progress.show();
        progress.setValue(0);

        //TODO: trying to load in a separate thread exposed problems with the signal/slots related to wiring up the GUI elements,
        // so I'll come back to this idea when there's time to refactor that part of the code (NEC::loadNetwork)
        //QFuture<int> future = QtConcurrent::run(load);
        //progress.setValue(future.result());
        networkEditor_->setVisibility(false);
        progress.setValue(load());
        networkEditor_->setVisibility(true);
      }
      file_ = file;

      QPointF center = findCenterOfNetworkFile(*file);
      networkEditor_->centerOn(center);

      GuiLogger::Instance().logInfoStd("File load done (" + filename_ + ").");
      return true;
    }
    else
    {
      if (!filename_.empty())
      {
        GuiLogger::Instance().logErrorStd("File load failed (" + filename_ + "): null xml returned.");
      }
    }
  }
  catch (ExceptionBase& e)
  {
    if (!filename_.empty())
      GuiLogger::Instance().logErrorStd("File load failed (" + filename_ + "): exception in load_xml, " + e.what());
  }
  catch (std::exception& ex)
  {
    if (!filename_.empty())
      GuiLogger::Instance().logErrorStd("File load failed(" + filename_ + "): exception in load_xml, " + ex.what());
  }
  catch (...)
  {
    if (!filename_.empty())
      GuiLogger::Instance().logErrorStd("File load failed(" + filename_ + "): Unknown exception in load_xml.");
  }
  return false;
}

int NetworkFileProcessCommand::guiProcess(const NetworkFileHandle& file)
{
  networkEditor_->clear();
  networkEditor_->loadNetwork(file);
  return static_cast<int>(file->network.modules.size()) + 1;
}

NetworkFileHandle FileOpenCommand::processXmlFile()
{
  return XMLSerializer::load_xml<NetworkFile>(filename_);
}

NetworkFileHandle FileImportCommand::processXmlFile()
{
  auto dtdpath = Core::Application::Instance().executablePath();
  const auto& modFactory = Core::Application::Instance().controller()->moduleFactory();
  LegacyNetworkIO lnio(dtdpath.string(), modFactory);
  return lnio.load_net(filename_);
}

bool RunPythonScriptCommandGui::execute()
{
  auto script = Application::Instance().parameters()->pythonScriptFile().get();
  SCIRunMainWindow::Instance()->runPythonScript(QString::fromStdString(script.string()));
  return true;
}

bool SetupDataDirectoryCommandGui::execute()
{
  auto dir = Application::Instance().parameters()->dataDirectory().get();
  LOG_DEBUG("Data dir set to: " << dir << std::endl);

  SCIRunMainWindow::Instance()->setDataDirectory(QString::fromStdString(dir.string()));

  return true;
}

NetworkSaveCommand::NetworkSaveCommand(const QString& filename, NetworkEditor* editor, SCIRunMainWindow* window) :
filename_(filename), editor_(editor), window_(window)
{}

bool NetworkSaveCommand::execute()
{
  std::string fileNameWithExtension = filename_.toStdString();
  if (!boost::algorithm::ends_with(fileNameWithExtension, ".srn5"))
    fileNameWithExtension += ".srn5";

  NetworkFileHandle file = editor_->saveNetwork();

  XMLSerializer::save_xml(*file, fileNameWithExtension, "networkFile");
  window_->setCurrentFile(QString::fromStdString(fileNameWithExtension));

  window_->statusBar()->showMessage("File saved: " + filename_, 2000);
  GuiLogger::Instance().logInfo("File save done: " + filename_);
  window_->setWindowModified(false);
  return true;
}
