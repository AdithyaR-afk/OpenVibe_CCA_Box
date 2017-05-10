#include "ovpCBoxAlgorithmSignalFlattener.h"


#include <Eigen/Dense>
#include <Eigen/Eigenvalues>

using namespace std;
using namespace Eigen;
using Eigen::MatrixXd;
using namespace OpenViBE;
using namespace OpenViBE::Kernel;
using namespace OpenViBE::Plugins;
 
using namespace OpenViBEPlugins;
using namespace OpenViBEPlugins::SignalProcessing;
typedef unsigned uint;
template<typename M>
M load_csv (const std::string & path) {
    std::ifstream indata;
    indata.open(path);
    std::string line;
    std::vector<double> values;
    uint rows = 0;
    while (std::getline(indata, line)) {
        std::stringstream lineStream(line);
        std::string cell;
        while (std::getline(lineStream, cell, ',')) {
            values.push_back(std::stod(cell));
        }
        ++rows;
    }
    return Map<const Matrix<typename M::Scalar, M::RowsAtCompileTime, M::ColsAtCompileTime, RowMajor>>(values.data(), rows, values.size()/rows);

}
 
OpenViBE::boolean CBoxAlgorithmSignalFlattener::initialize(void)
{
    // Signal stream decoder, connect to input stream 0
    m_oAlgo0_SignalDecoder.initialize(*this,0);
    // Stimulation stream decoder, connect to input stream 1
    m_oAlgo1_StimulationDecoder.initialize(*this, 1);
    // Signal stream encoder, connect to output stream 0
    m_oAlgo2_SignalEncoder.initialize(*this, 0);
 
    // We connect the Signal Input with the Signal Output so every chunk on the input will be copied to the output automatically.
    m_oAlgo2_SignalEncoder.getInputMatrix().setReferenceTarget(m_oAlgo0_SignalDecoder.getOutputMatrix());
    m_oAlgo2_SignalEncoder.getInputSamplingRate().setReferenceTarget(m_oAlgo0_SignalDecoder.getOutputSamplingRate());
 
    // Then we save the settings in variables:
    // the "Level" setting is at index 0, we can auto cast it from CString to float64
    m_f64LevelValue = FSettingValueAutoCast(*this->getBoxAlgorithmContext(), 0);
    // the "Trigger" setting is at index 1, we can auto cast it from CString to uint64
    m_ui64Trigger = FSettingValueAutoCast(*this->getBoxAlgorithmContext(), 1);
 
    // Finally, we disable the flat mode at start
    m_bFlatSignalRequested = false;
 
    return true;
}
/*******************************************************************************/
 
boolean CBoxAlgorithmSignalFlattener::uninitialize(void)
{
    m_oAlgo0_SignalDecoder.uninitialize();
    m_oAlgo1_StimulationDecoder.uninitialize();
    m_oAlgo2_SignalEncoder.uninitialize();
 
    return true;
}
/*******************************************************************************/
 
boolean CBoxAlgorithmSignalFlattener::processInput(uint32 ui32InputIndex)
{
    // tell the kernel we are ready to process !
    getBoxAlgorithmContext()->markAlgorithmAsReadyToProcess();
 
    return true;
}
/*******************************************************************************/
 int no = 0;
 int pos,neg,null = 0;
boolean CBoxAlgorithmSignalFlattener::process(void)
{
 
    // the static box context describes the box inputs, outputs, settings structures
    // IBox& l_rStaticBoxContext=this->getStaticBoxContext();
    // the dynamic box context describes the current state of the box inputs and outputs (i.e. the chunks)
    IBoxIO& l_rDynamicBoxContext=this->getDynamicBoxContext();
 
    // we will first check the pendoing stimulations to check if the trigger has been received.
    //iterate over all chunk on input 1 (Stimulations)
    for(uint32 i=0; i<l_rDynamicBoxContext.getInputChunkCount(1); i++)
    {
        //we decode the i:th chunk
        m_oAlgo1_StimulationDecoder.decode(i);
        if(m_oAlgo1_StimulationDecoder.isHeaderReceived())
        {
            // Header received. This happens only once when pressing "play".
            // nothing to do...
        }
        if(m_oAlgo1_StimulationDecoder.isBufferReceived())
        {
            // A buffer has been received, lets' check the stimulations inside
            IStimulationSet* l_pStimulations = m_oAlgo1_StimulationDecoder.getOutputStimulationSet();
            for(uint32 j=0; j<l_pStimulations->getStimulationCount(); j++)
            {
                uint64 l_ui64StimulationCode = l_pStimulations->getStimulationIdentifier(j);
                uint64 l_ui64StimulationDate = l_pStimulations->getStimulationDate(j);
                CString l_sStimulationName   = this->getTypeManager().getEnumerationEntryNameFromValue(OV_TypeId_Stimulation, l_ui64StimulationCode);
                //If the trigger is received, we switch the mode
                if(l_pStimulations->getStimulationIdentifier(j) == m_ui64Trigger)
                {
                    m_bFlatSignalRequested = !m_bFlatSignalRequested;
                    this->getLogManager() << LogLevel_Info << "Flat mode is now Hello Aditya["
                                                           << (m_bFlatSignalRequested ? "ENABLED" : "DISABLED")
                                                           << "]\n";
                }
                else
                {
                    this->getLogManager() << LogLevel_Warning << "Received an unrecognized trigger, with code ["
                                                              << l_ui64StimulationCode
                                                              << "], name ["
                                                              << l_sStimulationName
                                                              << "] and date ["
                                                              << time64(l_ui64StimulationDate)
                                                              << "]\n";
                }
            }
        }
        if(m_oAlgo1_StimulationDecoder.isEndReceived())
        {
            // End received. This happens only once when pressing "stop".
            // nothing to do...
        }
    }
 
					
					
					
					
    //now lets process the signal according to current mode
    //iterate over all chunk on input 0 (signal)
    for(uint32 i=0; i<l_rDynamicBoxContext.getInputChunkCount(0); i++)
    {
        // decode the chunk i
        m_oAlgo0_SignalDecoder.decode(i);
        // the decoder may have decoded 3 different parts : the header, a buffer or the end of stream.
        if(m_oAlgo0_SignalDecoder.isHeaderReceived())
        {
            // Header received. This happens only once when pressing "play".
            // Now we know the sampling rate of the signal, and we can get it thanks to:
            //uint64 l_uiSamplingFrequency = m_oAlgo0_SignalDecoder.getOutputSamplingRate();
 
            // Pass the header to the next boxes, by encoding a header 
            m_oAlgo2_SignalEncoder.encodeHeader();
            // send the output chunk containing the header. The dates are the same as the input chunk:
            l_rDynamicBoxContext.markOutputAsReadyToSend(0, l_rDynamicBoxContext.getInputChunkStartTime(0, i), l_rDynamicBoxContext.getInputChunkEndTime(0, i));
        }
        if(m_oAlgo0_SignalDecoder.isBufferReceived())
        {
            // Buffer received.
            IMatrix* l_pMatrix = m_oAlgo0_SignalDecoder.getOutputMatrix(); // the StreamedMatrix of samples.
            uint32 l_ui32ChannelCount = l_pMatrix->getDimensionSize(0);
            uint32 l_ui32SamplesPerChannel = l_pMatrix->getDimensionSize(1);
            float64* l_pBuffer = l_pMatrix->getBuffer();
            // according to current mode, we modify or not the buffers
           
                // we can access the sample i from channel j with: l_pBuffer[i+j*l_ui32SamplesPerChannel]
                // in our case we modify every samples, so we only do:
				uint32 j = 1;
				double max[3] = {0,0,0};
				float64 flag = 0;
                for(j=1; j<4; j++)//l_pMatrix->getBufferElementCount();
                {
                  
					Map<MatrixXd> az(l_pBuffer,500,4); 
					
					MatrixXd a = az.transpose();
					/*this->getLogManager() << LogLevel_Info <<"The first sample value is" <<a(0,0) <<"\n";
				  this->getLogManager() << LogLevel_Info <<"The first  sample value is" <<a(1,0) <<"\n";
				  this->getLogManager() << LogLevel_Info <<"The first sample value is" <<a(2,0) <<"\n";
				  this->getLogManager() << LogLevel_Info <<"The first sample value is" <<a(3,0) <<"\n"<<"\n";*/
					
					double pi=3.1416;

					double sinh[2000];
	

					double   fs=250;
					float tp=1/fs;
					float tpd=0;
					int wintime=2;
					double tparr[500];
					tpd=tp;
					MatrixXd b(4,500);
					int sti_f = 15;
					if(j==2)
					{
						sti_f = 10;
					}
					else if(j == 3)
					{
						sti_f = 6;
					}
					for (int i=0;i<500;i++)
					{
						b(0,i)=sin(2*pi*sti_f*tpd);
						b(1,i)=cos(2*pi*sti_f*tpd);
						b(2,i)=sin(2*pi*2*sti_f*tpd);
						b(3,i)=cos(2*pi*2*sti_f*tpd);
						//cout<<mref(0,i)<<mref(1,i)<<mref(2,i)<<mref(3,i);
						tpd=tpd+tp;
					}
					
				
	//MatrixXd b = load_csv<MatrixXd>("G:\wref.csv");
	
	//cout<<"matrix a"<<endl;
	//cout<<a<<endl;
	//cout<<"matrix b"<<endl;
	//cout<<b<<endl;
	
	int nrowsa=a.rows();
	int ncolsa=a.cols();
	int nrowsb=b.rows();
	int ncolsb=b.cols();
	
	/*this->getLogManager() << LogLevel_Info << "The no of columns are"<<ncolsa
																	<<"\n";*/
	//Matrix c is the concatenated version of a and b appended from the bottom
	MatrixXd c(nrowsa+nrowsb,ncolsa);
	//This for loop block does the concatenation.
     for(int s=0; s<(nrowsa+nrowsb);s++)
	{
		if(s<nrowsa){
		for(int j=0; j<ncolsa;j++)
		{
			c(s,j)=a(s,j);
			//cout<<c(s,j)<< "a"<<endl;
		}}
			if(s>nrowsa-1){
		for(int f=0; f<ncolsb;f++)
		{
			c(s,f)=b(s-nrowsb,f);
			//cout<<c(s,f)<< "b"<<endl;
			

		}}}// cout<<endl<<endl<<"concatenated matrix"<<endl<<c<<endl<<endl;
		
		
				  
		int nrowsc= c.rows();
	int rowsum= nrowsa+nrowsb;
	
	//Initialize a zero array v
	//ArrayXf k = ArrayXf::Zero(rowsum);
	ArrayXd v = ArrayXd::Zero(rowsum);
	
	//This block computes the rowwise mean of each row in matrix c.
	for(int i=0; i<(rowsum);i++)
	{
		for(int w=0;w<ncolsa;w++)
		{
	
	  v[i]=v[i]+c(i,w);

		}
		
		v[i]=v[i]/ncolsa;
		
		
	}
	
	//cout<<"mean array "<<endl<<v<<endl<<endl;
	//Initializing the covariance matrix 'cca' with zeroes.
	MatrixXd cca = MatrixXd::Zero(2*nrowsa,2*nrowsa);
	

	//cout<<"rowsum: "<<rowsum<<endl<<endl<<"cca"<<endl<<cca<<endl;
	//Computing the values of the covariance matrix.
	for(int x=0; x<rowsum;x++)
	{
		for(int y=0; y<(2*nrowsa);y++)
		{
			for(int z=0; z<ncolsa;z++)
			{
				cca(x,y) = cca(x,y) + (((c(x,z)-v[x])*(c(y,z)-v[y]))/(ncolsa-1));
			
			}
			
			
		}
	}

	//cout<<endl<<" covariance matrix 'cca' output"<<endl<<cca<<endl;
	
//Extracting each matrix from covariance matrix
	MatrixXd cxx = cca.block(0,0,nrowsa,nrowsa);
	MatrixXd cxy = cca.block(0,nrowsa,nrowsa,nrowsa);
	MatrixXd cyx = cca.block(nrowsa,0,nrowsa,nrowsa);
	MatrixXd cyy = cca.block(nrowsa,nrowsa,nrowsa,nrowsa);

	//Generating an identity matrix.
	MatrixXd id = MatrixXd::Identity(nrowsa,nrowsa);
	
	//In order to compute the inverse for cxx and cyy matrices, a small value is added to make sure that the inverse exists.
	MatrixXd id1= id*(0.000001);
	MatrixXd ncxx = cxx+id1;
	MatrixXd ncyy = cyy+id1;

	//Taking inverse.
	MatrixXd icxx= ncxx.inverse();
	MatrixXd icyy= ncyy.inverse();
   //cout <<endl<< "cxx "<<endl<<cxx<<endl;
   //cout <<endl<< "cxy"<<endl<<cxy<<endl;
   //cout <<endl<< "cyx"<<endl<<cyx<<endl;
   //cout <<endl<< "cyy"<<endl<<cyy<<endl;
  

	//cout<<endl<<endl<<"icxx"<< endl<<icxx<<endl;
	//cout<<"icyy"<<endl<<icyy<<endl;

	MatrixXd finpro = (icxx*cxy*icyy*cyx);
	//cout<<endl<<"finpro"<<endl<<finpro<<endl;
	//eigenvalue decomposition
	
   EigenSolver<MatrixXd> eig(finpro);
   
   //Printing the eigenvalues.
   int r=0;
  complex<double> eigenval=0;
   
  // cout<<"eigen values are  "<<endl;
   for(r=0;r<nrowsa;r++)
   {
	   complex<double> eigenval1 = sqrt(real(eig.eigenvalues()[r]));
	   
	   if(abs(eigenval) < abs(eigenval1))
	   {
	   eigenval=eigenval1;
	   
	   
	   }
	   
	//	 cout<<eigenval1<<endl;
   }
    max[j-1] = abs(eigenval);           
		  }
		  if(max[2] > max[1] && max[2] > max[0])
		  { this->getLogManager() << LogLevel_Info << "null \n";
			  pos = 0;
			  neg = 0;
		  }
		   else if(max[0] > max[1])
		  {  pos++;
			 neg = 0;
			this->getLogManager() << LogLevel_Info << "  The frequency is 15Hz"
													<<"\n";
		  }
		  else
		  {	 neg++;
			 pos = 0;
			this->getLogManager() << LogLevel_Info << "The frequency is 10Hz"<<"\n";
		  }
		  if(pos == 9)
		  {
													pos = 0;
													flag = -2000.0;
													 for(uint32 j=0; j<l_pMatrix->getBufferElementCount(); j++)
        {
            l_pBuffer[j] = flag;
        }
				
		  }
		  /*else if(neg == 15)
		  {
			  this->getLogManager() << LogLevel_Info << "  The frequency is 10Hz"
													<<"\n";
													neg = 0;
		  }*/
		  max[0] = 0;max[1] = 0;max[2]=0;
           
			
 
            // Encode the output buffer (modified or not) :
            m_oAlgo2_SignalEncoder.encodeBuffer();
            // and send it to the next boxes :
            l_rDynamicBoxContext.markOutputAsReadyToSend(0, l_rDynamicBoxContext.getInputChunkStartTime(0, i), l_rDynamicBoxContext.getInputChunkEndTime(0, i));
 
        }
        if(m_oAlgo0_SignalDecoder.isEndReceived())
        {
            // End of stream received. This happens only once when pressing "stop". Just pass it to the next boxes so they receive the message :
            m_oAlgo2_SignalEncoder.encodeEnd();
            l_rDynamicBoxContext.markOutputAsReadyToSend(0, l_rDynamicBoxContext.getInputChunkStartTime(0, i), l_rDynamicBoxContext.getInputChunkEndTime(0, i));
        }
 
        // The current input chunk has been processed, and automatically discarded thanks to the codec toolkit.
        // you don't need to call "l_rDynamicBoxContext.markInputAsDeprecated(0, i);"
    }
 
    return true;
}