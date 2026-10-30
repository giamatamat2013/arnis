//! Orchestrates 3D-model substitution: city models (real files) → 3DMR (external) → Wikidata (external) → custom archetypes.

use crate::args::Args;
use crate::coordinate_system::cartesian::XZBBox;
use crate::coordinate_system::geographic::LLBBox;
use crate::models_3d::{city_models, custom, three_dmr, wikidata};
use crate::osm_parser::ProcessedElement;
use crate::world_editor::WorldEditor;
use std::collections::HashSet;

pub struct Models3dPipeline {
    city: city_models::PrescanResult,
    three_dmr: three_dmr::PrescanResult,
    wikidata: wikidata::PrescanResult,
    stadium: custom::stadium::PrescanResult,
    union_suppressed: HashSet<(&'static str, u64)>,
}

impl Models3dPipeline {
    /// `already_suppressed` holds elements a landmark model already claims.
    pub fn prescan(
        elements: &[ProcessedElement],
        args: &Args,
        already_suppressed: &HashSet<(&'static str, u64)>,
        llbbox: LLBBox,
        xzbbox: &XZBBox,
    ) -> Self {
        let city = city_models::prescan(elements, already_suppressed, llbbox, xzbbox, args);

        let mut combined: HashSet<(&'static str, u64)> = already_suppressed.clone();
        combined.extend(city.suppressed_ids.iter().copied());
        let three_dmr = three_dmr::prescan(elements, &combined, args.rotation);
        combined.extend(three_dmr.suppressed_ids.iter().copied());
        let wikidata = wikidata::prescan(elements, &combined, args.rotation, args.scale);
        combined.extend(wikidata.suppressed_ids.iter().copied());

        let stadium = custom::stadium::prescan(elements, &combined, args.scale);
        combined.extend(stadium.suppressed_ids.iter().copied());

        Self {
            city,
            three_dmr,
            wikidata,
            stadium,
            union_suppressed: combined,
        }
    }

    pub fn suppressed(&self) -> &HashSet<(&'static str, u64)> {
        &self.union_suppressed
    }

    /// Regions each 3D placement may write to, in `place` order, so stream-to-disk can keep
    /// them resident until it is placed. wikidata/stadium use their size caps; 3DMR
    /// (uncapped) uses a generous assumed extent.
    pub fn placement_regions(&self, scale: f64) -> Vec<Vec<(i32, i32)>> {
        let mut out = self.city.placement_regions(scale);
        out.extend(self.three_dmr.placement_regions(scale));
        out.extend(self.wikidata.placement_regions(scale));
        out.extend(self.stadium.placement_regions(scale));
        out
    }

    pub fn placement_count(&self) -> usize {
        self.city.placement_count()
            + self.three_dmr.placement_count()
            + self.wikidata.placement_count()
            + self.stadium.placement_count()
    }

    /// Places `indices` (into `placement_regions`, ascending).
    pub fn place(&self, editor: &mut WorldEditor, args: &Args, indices: &[usize]) {
        let city = self.city.placement_count();
        let dmr = city + self.three_dmr.placement_count();
        let wiki = dmr + self.wikidata.placement_count();
        let range = |lo: usize, hi: usize| -> Vec<usize> {
            indices
                .iter()
                .filter(|&&i| (lo..hi).contains(&i))
                .map(|&i| i - lo)
                .collect()
        };
        city_models::place(editor, args, &self.city, &range(0, city));
        three_dmr::place_three_dmr_models(editor, args, &self.three_dmr, &range(city, dmr));
        wikidata::place_wikidata_models(editor, args, &self.wikidata, &range(dmr, wiki));
        custom::stadium::place_stadium_models(
            editor,
            args,
            &self.stadium,
            &range(wiki, usize::MAX),
        );
    }
}
