#include "LocalizerUtils.h"

#include "StringWordWriter.h"

#include <fstream>


// Writes one entry
void WriteLocalizerHistoryEntry(const Localizer::HistoryEntry& historyEntry, StringWordWriter& writer)
{
	const char* pPoseFmt = "%.5f";

	writer.Write("HISTORY_ENTRY");

	writer.Write("ENTRY_ID");
	writer.Write(historyEntry.m_Id);

	writer.Write("AFTER_RESAMPLING");
	writer.Write<uint>(static_cast<uint>(historyEntry.m_bAfterResampling));
	writer.WriteNewLine();

	writer.Write("COMMON_POSE_HISTORY");
	writer.Write("N_POSES");
	writer.Write<uint>(static_cast<uint>(historyEntry.m_CommonPoseHistory.size()));
	writer.WriteNewLine();
	{
		uint newlineCounter = 0;
		for (const Pose& commonPose : historyEntry.m_CommonPoseHistory)
		{
			writer.Write("POSE");
			writer.Write(commonPose.x, pPoseFmt);
			writer.Write(commonPose.y, pPoseFmt);
			writer.Write(commonPose.dir, pPoseFmt);

			newlineCounter++;
			if (newlineCounter > 8)
			{
				writer.WriteNewLine();
				newlineCounter = 0;
			}
		}
	}
	writer.WriteNewLine();

	writer.Write("PARTICLES");
	writer.Write("N_PARTICLES");
	writer.Write<uint>(static_cast<uint>(historyEntry.m_Particles.size()));
	writer.WriteNewLine();

	for (const Localizer::HistoryEntry::ParticleEntry& p : historyEntry.m_Particles)
	{
		writer.Write("PTCL");
		writer.Write("POSE");
		writer.Write(p.m_Pose.x, pPoseFmt);
		writer.Write(p.m_Pose.y, pPoseFmt);
		writer.Write(p.m_Pose.dir, pPoseFmt);

		writer.Write("WT");
		writer.Write(p.m_Weight);

		writer.Write("UNIQUE_POS_HIST");
		writer.Write("N_POSES");
		writer.Write<uint>(static_cast<uint>(p.m_UniquePoseHistory.size()));

		{
			uint newlineCounter = 0;
			for (const Pose& uniquePose : p.m_UniquePoseHistory)
			{
				writer.Write("POSE");
				writer.Write(uniquePose.x, pPoseFmt);
				writer.Write(uniquePose.y, pPoseFmt);
				writer.Write(uniquePose.dir, pPoseFmt);

				newlineCounter++;
				if (newlineCounter > 8)
				{
					writer.WriteNewLine();
					newlineCounter = 0;
				}
			}
		}

		writer.WriteNewLine();
	}
}

bool LocalizerUtils::WriteLocalizerHistoryToFile(const Localizer& localizer, const char* pFilePath, uint nEntriesToWrite)
{
	// Remember binary flag! Otherwise size measurement goes wrong, because of CRLF conversions.
	std::ofstream f(pFilePath, std::ios_base::out | std::ios_base::binary);
	if (!f.is_open())
		return false;

	std::string outString;
	StringWordWriter writer(outString);

	writer.Write("LOCALIZER_HISTORY");

	writer.Write("VERSION");
	writer.Write<uint>(2);

	const auto& historyEntries = localizer.GetHistory();

	writer.WriteNewLine();
	writer.Write("HISTORY_ENTRIES");
	writer.WriteNewLine();

	// Select which entries to write
	std::vector<const Localizer::HistoryEntry*> entriesToWrite;
	{
		const uint batchSize = 4;
		const uint nBatches = nEntriesToWrite / batchSize;
		uint entryId = 0;
		uint batchCounter = 0;
		uint batchSizeCounter = 0;
		while (entryId < (uint)historyEntries.size() && (uint)entriesToWrite.size() < nEntriesToWrite)
		{
			entriesToWrite.push_back(&historyEntries[entryId]);
			batchSizeCounter++;
			if (batchSizeCounter >= batchSize)
			{
				batchSizeCounter = 0;
				batchCounter++;
				entryId = batchCounter * (uint)historyEntries.size() / nBatches;
			}
			else
			{
				entryId++;
			}
		}

		// Always write last history entry
		if (!historyEntries.empty())
		{
			const Localizer::HistoryEntry* pLastEntry = &historyEntries.back();
			if (entriesToWrite.back() != pLastEntry)
				entriesToWrite.push_back(pLastEntry);
		}
	}

	for (const Localizer::HistoryEntry* pHistoryEntry: entriesToWrite)
	{
		WriteLocalizerHistoryEntry(*pHistoryEntry, writer);
	}

	f.write(outString.data(), outString.size());

    return true;
}
