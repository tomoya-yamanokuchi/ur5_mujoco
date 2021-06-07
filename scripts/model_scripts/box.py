import copy
import dataclasses


@dataclasses.dataclass
class Basket:
    wall_thicknes: float
    bottom_size  : tuple
    wall_height  : float
    rgba_alpha   : float

    def __post_init__(self):
        self.xml = self.create()


    def template(self):
        xml = '''
        <body name="basket" pos="-0.3 0.8 0.01">
            <geom name="basket_bottom" type="box" size="basket_bottom_x basket_bottom_y 0.005" pos=" 0.00 0 0" mass="0.01" solimp="1 1 0 0.5 2" euler="0 0.00 0" friction="2 2 2" rgba="0 0.7 1 1"/>
            <body name="basket_wall">
                <geom name="basket_wall1"  type="box" size="wall_height     basket_bottom_y wall_thickness" pos=" basket_bottom_x                0 0.045" mass="0.01" solimp="1 1 0 0.5 2" euler="0 1.57 0" friction="2 2 2" rgba="0 0.7 1 rgba_alpha"/>
                <geom name="basket_wall2"  type="box" size="wall_height     basket_bottom_y wall_thickness" pos="-basket_bottom_x                0 0.045" mass="0.01" solimp="1 1 0 0.5 2" euler="0 1.57 0" friction="2 2 2" rgba="0 0.7 1 rgba_alpha"/>
                <geom name="basket_wall3"  type="box" size="basket_bottom_x wall_height wall_thickness" pos="               0 -basket_bottom_y 0.045" mass="0.01" solimp="1 1 0 0.5 2" euler="1.57 0 0" friction="2 2 2"     rgba="0 0.7 1 rgba_alpha"/>
                <geom name="basket_wall4"  type="box" size="basket_bottom_x wall_height wall_thickness" pos="               0  basket_bottom_y 0.045" mass="0.01" solimp="1 1 0 0.5 2" euler="1.57 0 0" friction="2 2 2"     rgba="0 0.7 1 rgba_alpha"/>
            </body>
            <freejoint/>
        </body>
        '''
        return copy.deepcopy(xml)


    def create(self):
        template = self.template()
        xml      = template
        replace_set = {
            'wall_thickness' : self.wall_thicknes,
            'basket_bottom_x': self.bottom_size[0],
            'basket_bottom_y': self.bottom_size[1],
            'wall_height'    : self.wall_height,
            'rgba_alpha'     : self.rgba_alpha
        }
        for oldStatemanet, newValue in replace_set.items():
            xml = xml.replace(oldStatemanet, str(newValue))
        return xml



if __name__ == '__main__':
    import xml.etree.ElementTree as ET

    basket = Basket(
        wall_thicknes  = 0.005,
        bottom_size    = (0.07, 0.07),
        wall_height    = 0.05,
        rgba_alpha     = 0.6
    )

    # xml = ET.parse()
    # with open('./model/ur5_abstract.xml', 'w') as f:
        # print(f)
    print(basket.xml)




